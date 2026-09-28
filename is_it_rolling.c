/*
 * Is it rolling ?  -  listen-only Sub-GHz rolling code estimator
 *
 * The app tunes the CC1101 to ONE frequency, asks you to press the SAME remote
 * button N times, captures the first frame of every press as a raw "chip"
 * sequence (pulse widths quantised to a base time element) and compares the
 * frames with each other.
 *
 *   - frames identical                       -> fixed code
 *   - stable block + block of random-looking
 *     bits that change on every press        -> rolling / hopping code
 *
 * There is NO transmit, NO emulation and NOTHING is saved to the SD card.
 * Only stock SDK APIs (furi, subghz_devices, gui, input, notification).
 */

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>
#include <notification/notification_messages.h>
#include <lib/subghz/devices/devices.h>
#include <lib/subghz/devices/preset.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------- tunables ---------------------------------- */
#define MIN_PRESSES     2
#define MAX_PRESSES     8
#define DEFAULT_PRESSES 5

#define MAX_PULSES       512 /* pulses kept per frame                      */
#define MAX_CHIPS        768 /* chips (time-quantised bits) kept per frame */
#define MIN_FRAME_PULSES 30
#define FRAME_GAP_US     8000 /* a LOW longer than this ends a frame        */
#define PRESS_IDLE_MS    500  /* silence needed before a new press counts   */

static const uint32_t freq_list[] = {
    315000000,
    318000000,
    390000000,
    433420000,
    433920000,
    434420000,
    868350000,
    915000000,
};
#define FREQ_COUNT (sizeof(freq_list) / sizeof(freq_list[0]))
#define FREQ_DEFAULT_IDX 4 /* 433.92 MHz */

/* ------------------------------ types ----------------------------------- */
typedef enum {
    ScreenSetup,
    ScreenListen,
    ScreenResult,
} Screen;

typedef struct {
    uint8_t bits[MAX_CHIPS / 8];
    uint16_t len;
} Frame;

typedef struct {
    int prob; /* 0..100 */
    uint8_t frames;
    uint16_t len;
    int var_pct;
    int fixed_pct;
} Analysis;

typedef struct {
    Screen screen;
    uint8_t cursor; /* setup row */
    uint8_t freq_idx;
    bool fm; /* false = AM650, true = FM476 */
    uint8_t target;

    /* capture state */
    bool rx_active;
    bool collecting;
    bool gap_seen;
    bool have_press;
    uint8_t count;
    uint16_t repeats;
    uint16_t n_pulses;
    uint32_t te;
    uint32_t last_frame_tick;
    float rssi;
    char status[32];

    uint32_t pulses[MAX_PULSES];
    uint32_t scratch[MAX_PULSES];
    Frame frames[MAX_PRESSES];
    Analysis res;

    /* infrastructure */
    const SubGhzDevice* device;
    FuriMutex* mutex;
    FuriMessageQueue* queue;
    FuriStreamBuffer* stream;
    ViewPort* vp;
    Gui* gui;
    NotificationApp* notif;
} App;

/* --------------------------- bit helpers -------------------------------- */
static inline uint8_t get_bit(const Frame* f, uint16_t i) {
    return (f->bits[i >> 3] >> (i & 7)) & 1;
}
static inline void set_bit(Frame* f, uint16_t i) {
    f->bits[i >> 3] |= (uint8_t)(1 << (i & 7));
}

/* ----------------------------- radio ------------------------------------ */
/* Runs in interrupt context: only push the pulse into a stream buffer. */
static void rx_capture_cb(bool level, uint32_t duration, void* ctx) {
    App* a = ctx;
    uint32_t v = (duration & 0x7FFFFFFFu) | (level ? 0x80000000u : 0u);
    furi_stream_buffer_send(a->stream, &v, sizeof(v), 0);
}

static void rx_start(App* a) {
    uint32_t freq = freq_list[a->freq_idx];
    furi_stream_buffer_reset(a->stream);

    subghz_devices_begin(a->device);
    subghz_devices_reset(a->device);
    subghz_devices_idle(a->device);
    subghz_devices_load_preset(
        a->device,
        a->fm ? FuriHalSubGhzPreset2FSKDev476Async : FuriHalSubGhzPresetOok650Async,
        NULL);
    subghz_devices_set_frequency(a->device, freq);
    subghz_devices_flush_rx(a->device);
    subghz_devices_set_rx(a->device);
    subghz_devices_start_async_rx(a->device, rx_capture_cb, a);
    a->rx_active = true;
}

static void rx_stop(App* a) {
    if(!a->rx_active) return;
    subghz_devices_stop_async_rx(a->device);
    subghz_devices_idle(a->device);
    subghz_devices_sleep(a->device);
    subghz_devices_end(a->device);
    a->rx_active = false;
}

/* --------------------------- signal analysis ---------------------------- */
static int cmp_u32(const void* x, const void* y) {
    uint32_t a = *(const uint32_t*)x, b = *(const uint32_t*)y;
    return (a > b) - (a < b);
}

/* Base time element = 25th percentile of the pulse widths of the frame. */
static uint32_t estimate_te(App* a) {
    memcpy(a->scratch, a->pulses, a->n_pulses * sizeof(uint32_t));
    qsort(a->scratch, a->n_pulses, sizeof(uint32_t), cmp_u32);
    return a->scratch[a->n_pulses / 4];
}

/* Reject noise: most pulses must be close to an integer multiple of te. */
static bool frame_is_clean(const App* a, uint32_t te) {
    uint32_t good = 0;
    for(uint16_t i = 0; i < a->n_pulses; i++) {
        uint32_t d = a->pulses[i];
        uint32_t rem = d % te;
        uint32_t err = rem < (te - rem) ? rem : (te - rem);
        uint32_t tol = (te * 3) / 10 + d / 20;
        if(err <= tol) good++;
    }
    return good * 10 >= (uint32_t)a->n_pulses * 7;
}

static void build_frame(Frame* f, const uint32_t* d, uint16_t n, uint32_t te) {
    memset(f, 0, sizeof(*f));
    bool level = true;
    for(uint16_t i = 0; i < n; i++) {
        uint32_t q = (d[i] + te / 2) / te;
        if(q == 0) q = 1;
        if(q > 32) q = 32;
        for(uint32_t k = 0; k < q; k++) {
            if(f->len >= MAX_CHIPS) return;
            if(level) set_bit(f, f->len);
            f->len++;
        }
        level = !level;
    }
}

/*
 * Heuristic score. Three pieces of evidence:
 *  a) how many chip positions change between presses
 *     (rolling: ~10-80 %, noise: ~0 %, everything: probably different signals)
 *  b) how "random" the changing zone is (pairwise flip density)
 *  c) is there a stable block (serial number) that stays identical
 * Then pulled towards 50 % when only few presses were captured.
 */
static void analyze(App* a) {
    Analysis* r = &a->res;
    memset(r, 0, sizeof(*r));
    uint8_t n = a->count;
    uint16_t len = a->frames[0].len;
    for(uint8_t f = 1; f < n; f++)
        if(a->frames[f].len < len) len = a->frames[f].len;
    r->frames = n;
    r->len = len;
    if(len < 32 || n < 2) {
        r->prob = 50;
        return;
    }

    uint16_t first = 0xFFFF, last = 0, varc = 0;
    for(uint16_t i = 0; i < len; i++) {
        uint8_t ones = 0;
        for(uint8_t f = 0; f < n; f++) ones += get_bit(&a->frames[f], i);
        if(ones != 0 && ones != n) {
            varc++;
            if(first == 0xFFFF) first = i;
            last = i;
        }
    }
    int var10 = (int)((uint32_t)varc * 1000 / len);
    r->var_pct = (int)((uint32_t)varc * 100 / len);
    r->fixed_pct = 100 - r->var_pct;

    if(var10 < 10) { /* frames are (almost) identical -> fixed code */
        r->prob = 3;
        return;
    }

    uint32_t diff = 0, pairs = 0;
    for(uint8_t x = 0; x < n; x++) {
        for(uint8_t y = x + 1; y < n; y++) {
            pairs++;
            for(uint16_t i = first; i <= last; i++)
                diff += get_bit(&a->frames[x], i) ^ get_bit(&a->frames[y], i);
        }
    }
    uint32_t window = (uint32_t)(last - first + 1) * pairs;
    int density = (int)(diff * 100 / window);

    int sa; /* amount of variation */
    if(var10 < 50)
        sa = var10 - 10; /* 0..40   */
    else if(var10 < 150)
        sa = 40 + (var10 - 50) * 60 / 100; /* 40..100 */
    else if(var10 <= 800)
        sa = 100;
    else
        sa = 100 - (var10 - 800) * 60 / 200; /* 100..40 */

    int sb = density * 10; /* >=10 % flip density -> 100 */
    if(sb > 100) sb = 100;
    int sc = r->fixed_pct * 100 / 25; /* >=25 % stable -> 100 */
    if(sc > 100) sc = 100;

    int raw = (40 * sa + 30 * sb + 30 * sc) / 100;
    int conf = 50 + (n - 2) * 13;
    if(conf > 100) conf = 100;
    int prob = 50 + (raw - 50) * conf / 100;
    if(prob < 0) prob = 0;
    if(prob > 100) prob = 100;
    r->prob = prob;
}

static void finish_capture(App* a) {
    analyze(a);
    rx_stop(a);
    a->screen = ScreenResult;
    notification_message(a->notif, &sequence_success);
}

/* ------------------------- frame / press logic -------------------------- */
static void finish_frame(App* a) {
    if(a->n_pulses < MIN_FRAME_PULSES) return;

    uint32_t te = (a->count == 0) ? estimate_te(a) : a->te;
    if(te < 80 || te > 3000) return;
    if(!frame_is_clean(a, te)) return;

    uint32_t now = furi_get_tick();
    if(a->have_press && (now - a->last_frame_tick) < PRESS_IDLE_MS) {
        /* repeat of the frame within the same press: ignore, extend hold */
        a->last_frame_tick = now;
        a->repeats++;
        return;
    }

    /* first frame of a new press */
    if(a->count == 0) a->te = te;
    build_frame(&a->frames[a->count], a->pulses, a->n_pulses, a->te);
    a->count++;
    a->have_press = true;
    a->last_frame_tick = now;
    a->repeats = 0;
    snprintf(
        a->status, sizeof(a->status), "Got #%u (%u chips)", a->count,
        a->frames[a->count - 1].len);

    if(a->count >= a->target) {
        finish_capture(a);
    } else {
        notification_message(a->notif, &sequence_blink_blue_10);
    }
}

static void handle_pulse(App* a, bool level, uint32_t dur) {
    if(!a->collecting) {
        if(!level && dur >= FRAME_GAP_US) {
            a->gap_seen = true;
        } else if(level && a->gap_seen && dur < FRAME_GAP_US) {
            a->collecting = true;
            a->n_pulses = 0;
            a->pulses[a->n_pulses++] = dur;
        }
        return;
    }

    if(!level && dur >= FRAME_GAP_US) { /* end of frame */
        finish_frame(a);
        a->collecting = false;
        a->gap_seen = true;
        return;
    }
    if(level && dur >= FRAME_GAP_US) { /* absurdly long HIGH: noise/carrier */
        a->collecting = false;
        a->gap_seen = false;
        return;
    }
    if(a->n_pulses >= MAX_PULSES) { /* too long, keep what we have */
        finish_frame(a);
        a->collecting = false;
        a->gap_seen = false;
        return;
    }
    a->pulses[a->n_pulses++] = dur;
}

static void reset_session(App* a) {
    a->count = 0;
    a->repeats = 0;
    a->collecting = false;
    a->gap_seen = false;
    a->have_press = false;
    a->te = 0;
    a->n_pulses = 0;
    a->rssi = -120.0f;
    snprintf(a->status, sizeof(a->status), "Waiting for signal...");
}

static void start_listening(App* a) {
    uint32_t freq = freq_list[a->freq_idx];
    if(!subghz_devices_is_frequency_valid(a->device, freq)) {
        snprintf(a->status, sizeof(a->status), "Frequency invalid");
        return;
    }
    reset_session(a);
    rx_start(a);
    a->screen = ScreenListen;
}

/* ------------------------------- UI ------------------------------------- */
static void fmt_freq(char* buf, size_t n, uint32_t f) {
    snprintf(buf, n, "%lu.%02lu", f / 1000000UL, (f % 1000000UL) / 10000UL);
}

static void draw_setup(Canvas* c, App* a) {
    char buf[40], fb[16];
    canvas_set_font(c, FontPrimary);
    canvas_draw_str_aligned(c, 64, 1, AlignCenter, AlignTop, "Is it rolling ?");
    canvas_set_font(c, FontSecondary);

    fmt_freq(fb, sizeof(fb), freq_list[a->freq_idx]);
    const int y[3] = {24, 35, 46};
    for(uint8_t i = 0; i < 3; i++)
        if(a->cursor == i) canvas_draw_str(c, 0, y[i], ">");

    canvas_draw_str(c, 8, y[0], "Freq");
    snprintf(buf, sizeof(buf), "< %s MHz >", fb);
    canvas_draw_str_aligned(c, 126, y[0], AlignRight, AlignBottom, buf);

    canvas_draw_str(c, 8, y[1], "Mod");
    snprintf(buf, sizeof(buf), "< %s >", a->fm ? "FM476" : "AM650");
    canvas_draw_str_aligned(c, 126, y[1], AlignRight, AlignBottom, buf);

    canvas_draw_str(c, 8, y[2], "Presses");
    snprintf(buf, sizeof(buf), "< %u >", a->target);
    canvas_draw_str_aligned(c, 126, y[2], AlignRight, AlignBottom, buf);

    canvas_draw_str(c, 0, 62, "OK: listen");
    canvas_draw_str_aligned(c, 127, 62, AlignRight, AlignBottom, "Back: exit");
}

static void draw_listen(Canvas* c, App* a) {
    char buf[48], fb[16];
    fmt_freq(fb, sizeof(fb), freq_list[a->freq_idx]);

    canvas_set_font(c, FontPrimary);
    snprintf(buf, sizeof(buf), "Listening %s MHz", fb);
    canvas_draw_str(c, 0, 10, buf);

    canvas_set_font(c, FontSecondary);
    canvas_draw_str(c, 0, 23, "Press the SAME button");
    snprintf(buf, sizeof(buf), "Press %u/%u  (repeats: %u)", a->count, a->target, a->repeats);
    canvas_draw_str(c, 0, 34, buf);

    canvas_draw_frame(c, 0, 38, 128, 8);
    int w = (int)a->count * 124 / (int)a->target;
    if(w > 0) canvas_draw_box(c, 2, 40, w, 4);

    canvas_draw_str(c, 0, 58, a->status);
    snprintf(buf, sizeof(buf), "%d dBm", (int)a->rssi);
    canvas_draw_str_aligned(c, 127, 63, AlignRight, AlignBottom, buf);
}

static void draw_result(Canvas* c, App* a) {
    char buf[48];
    const Analysis* r = &a->res;

    canvas_set_font(c, FontSecondary);
    canvas_draw_str(c, 0, 8, "Rolling code probability");

    canvas_set_font(c, FontBigNumbers);
    snprintf(buf, sizeof(buf), "%d", r->prob);
    canvas_draw_str(c, 2, 36, buf);
    uint16_t w = canvas_string_width(c, buf);
    canvas_set_font(c, FontPrimary);
    canvas_draw_str(c, 2 + w + 2, 36, "%");

    const char *l1, *l2;
    if(r->prob >= 60) {
        l1 = "Likely";
        l2 = "ROLLING";
    } else if(r->prob <= 20) {
        l1 = "Likely";
        l2 = "FIXED";
    } else {
        l1 = "Inconclusive";
        l2 = "Retry";
    }
    canvas_draw_str(c, 70, 22, l1);
    canvas_draw_str(c, 70, 35, l2);

    canvas_set_font(c, FontSecondary);
    snprintf(
        buf, sizeof(buf), "%u frames, %u chips, %d%% varies", r->frames, r->len, r->var_pct);
    canvas_draw_str(c, 0, 48, buf);
    canvas_draw_str(c, 0, 61, "OK: retry");
    canvas_draw_str_aligned(c, 127, 63, AlignRight, AlignBottom, "Back: menu");
}

static void draw_cb(Canvas* canvas, void* ctx) {
    App* a = ctx;
    furi_mutex_acquire(a->mutex, FuriWaitForever);
    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);
    switch(a->screen) {
    case ScreenSetup:
        draw_setup(canvas, a);
        break;
    case ScreenListen:
        draw_listen(canvas, a);
        break;
    case ScreenResult:
        draw_result(canvas, a);
        break;
    }
    furi_mutex_release(a->mutex);
}

static void input_cb(InputEvent* event, void* ctx) {
    App* a = ctx;
    furi_message_queue_put(a->queue, event, 0);
}

/* returns false when the app should exit */
static bool handle_input(App* a, const InputEvent* e) {
    if(e->type != InputTypeShort && e->type != InputTypeRepeat) return true;
    bool keep = true;
    furi_mutex_acquire(a->mutex, FuriWaitForever);

    switch(a->screen) {
    case ScreenSetup:
        if(e->key == InputKeyUp) {
            a->cursor = a->cursor ? a->cursor - 1 : 2;
        } else if(e->key == InputKeyDown) {
            a->cursor = (a->cursor + 1) % 3;
        } else if(e->key == InputKeyLeft || e->key == InputKeyRight) {
            int dir = (e->key == InputKeyRight) ? 1 : -1;
            if(a->cursor == 0) {
                a->freq_idx = (uint8_t)((a->freq_idx + (int)FREQ_COUNT + dir) % (int)FREQ_COUNT);
            } else if(a->cursor == 1) {
                a->fm = !a->fm;
            } else {
                int t = (int)a->target + dir;
                if(t < MIN_PRESSES) t = MIN_PRESSES;
                if(t > MAX_PRESSES) t = MAX_PRESSES;
                a->target = (uint8_t)t;
            }
        } else if(e->key == InputKeyOk) {
            start_listening(a);
        } else if(e->key == InputKeyBack) {
            keep = false;
        }
        break;

    case ScreenListen:
        if(e->key == InputKeyBack) {
            rx_stop(a);
            a->screen = ScreenSetup;
        } else if(e->key == InputKeyOk) {
            reset_session(a); /* restart counting */
        }
        break;

    case ScreenResult:
        if(e->key == InputKeyOk) {
            start_listening(a);
        } else if(e->key == InputKeyBack) {
            a->screen = ScreenSetup;
        }
        break;
    }

    furi_mutex_release(a->mutex);
    view_port_update(a->vp);
    return keep;
}

static void process_stream(App* a) {
    uint32_t v;
    bool touched = false;
    furi_mutex_acquire(a->mutex, FuriWaitForever);
    while(a->rx_active && furi_stream_buffer_receive(a->stream, &v, sizeof(v), 0) == sizeof(v)) {
        uint8_t before = a->count;
        uint16_t rep = a->repeats;
        handle_pulse(a, (v >> 31) != 0, v & 0x7FFFFFFFu);
        if(a->count != before || a->repeats != rep) touched = true;
    }
    furi_mutex_release(a->mutex);
    if(touched) view_port_update(a->vp);
}

/* ------------------------------ main ------------------------------------ */
int32_t is_it_rolling_app(void* p) {
    UNUSED(p);

    App* a = malloc(sizeof(App));
    memset(a, 0, sizeof(App));
    a->screen = ScreenSetup;
    a->freq_idx = FREQ_DEFAULT_IDX;
    a->target = DEFAULT_PRESSES;
    a->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    a->queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    a->stream = furi_stream_buffer_alloc(4096, sizeof(uint32_t));
    a->vp = view_port_alloc();
    view_port_draw_callback_set(a->vp, draw_cb, a);
    view_port_input_callback_set(a->vp, input_cb, a);
    a->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(a->gui, a->vp, GuiLayerFullscreen);
    a->notif = furi_record_open(RECORD_NOTIFICATION);

    subghz_devices_init();
    a->device = subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_INT_NAME);

    bool running = true;
    uint32_t last_rssi = 0;
    InputEvent ev;

    while(running) {
        if(furi_message_queue_get(a->queue, &ev, 5) == FuriStatusOk) {
            running = handle_input(a, &ev);
        }
        if(a->rx_active) {
            process_stream(a);
            uint32_t now = furi_get_tick();
            if(now - last_rssi > 250) {
                last_rssi = now;
                furi_mutex_acquire(a->mutex, FuriWaitForever);
                if(a->rx_active) a->rssi = subghz_devices_get_rssi(a->device);
                furi_mutex_release(a->mutex);
                view_port_update(a->vp);
            }
        }
    }

    rx_stop(a);
    subghz_devices_deinit();
    gui_remove_view_port(a->gui, a->vp);
    view_port_free(a->vp);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);
    furi_stream_buffer_free(a->stream);
    furi_message_queue_free(a->queue);
    furi_mutex_free(a->mutex);
    free(a);
    return 0;
}

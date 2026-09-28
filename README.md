# Is it rolling ?

Listen-only Flipper Zero app. It captures N presses of the same remote button and estimates the probability that the remote uses a rolling code. No TX, no emulation, nothing saved.

## Build (stock SDK)

Standalone with ufbt:

    pip install ufbt
    cd is_it_rolling
    ufbt            # builds dist/is_it_rolling.fap
    ufbt launch     # builds, uploads and runs on a connected Flipper

Or copy the folder into `applications_user/` of the official firmware repo and run `./fbt fap_is_it_rolling`.

## Use

1. Pick the frequency (Left/Right), modulation (AM650 by default, FM476 for FSK remotes) and number of presses (2-8).
2. OK to listen. Press the SAME button, once per press, releasing fully between presses (about half a second).
3. After N presses you get a rolling-code probability.

Holding the button sends repeated copies of one frame. The app counts these as a single press.

## How the score works

Each press's first frame is captured as a time-quantised chip sequence and compared with the others.

- Frames identical on every press: fixed code, about 3 %.
- A stable block (serial number) plus a block of random-looking bits that changes on every press: rolling or hopping code, high score.
- Only a few low bits change: could be a simple counter, so the score is middling.
- Confidence is pulled toward 50 % when only 2-3 presses were captured. 5 or more is recommended.

This is a heuristic, not a protocol decoder. Results can be wrong if you press different buttons, are too far from or too close to the remote, or if the remote uses a time-based or otherwise unusual scheme.

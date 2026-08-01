# OpenMV Ball Vision Design

Date: 2026-08-01

## Goal

Implement a high-frame-rate OpenMV script that detects the bright horizontal water pipe, tracks the small dark steel ball inside it, estimates the ball offset distance from the pipe center, and sends robust UART data to the STM32 controller.

## Confirmed Interface

- OpenMV UART: `UART(3, 9600)`.
- Valid detection packet: `$B,<mm>#`.
- Lost packet: `$L#`.
- The payload is whole millimetres. The STM32 side converts it to 0.1 mm internally.

## Recommended Algorithm

Use grayscale QQVGA processing for speed and lighting robustness.

1. Lock the pipe ROI from row brightness projection. The pipe is the brightest horizontal band in the sample images.
2. Re-lock the pipe periodically or after repeated lost frames, not every frame.
3. Search only inside the pipe ROI. When the ball was found recently, search a small ROI around the predicted x position first.
4. Detect the ball as a local dark blob using ROI-relative thresholds based on histogram percentiles.
5. Reject noise using blob area, width, height, density, aspect ratio, and edge-touch checks.
6. Smooth accepted x positions with a lightweight IIR filter.
7. Estimate distance as `(ball_x - CENTER_X) * MM_PER_PIXEL`.
8. Send `$B,<integer_mm>#` at a fixed rate when valid; send `$L#` when the ball is lost long enough.

## Lighting Robustness

- Use grayscale to avoid color shifts.
- Disable white balance.
- Use controlled exposure and gain defaults, with constants exposed for tuning.
- Use local histogram thresholds rather than fixed pixel thresholds.
- Recompute pipe ROI from row brightness when the scene changes.

## High Frame Rate Choices

- QQVGA frame size.
- Avoid full-frame pixel loops during normal tracking.
- Keep expensive pipe search on a schedule.
- Use small tracking ROI around the previous ball position.
- Keep drawing/debug output behind a `DEBUG_DRAW` switch.

## Field Debug Mode

- Let automatic exposure and gain settle during startup, then lock both values so
  the image is bright enough without frame-to-frame brightness pumping.
- Enable debug drawing while tuning: draw the pipe ROI, the ball bounding box,
  the ball center cross, and the image center reference line.
- Print one status line every 100 ms instead of every frame. A detected line
  includes FPS, ball `(x, y)`, integer millimetre offset, pipe ROI, and adaptive
  threshold. A lost line explicitly reports `BALL=LOST` with the same diagnostic
  context.
- Keep UART transmission independent of terminal printing, so diagnostics do not
  alter the STM32 packet cadence.

## Field Detection Correction

The live IDE frame showed that pipe localization was correct, but the inner ball
search ROI was only 15 pixels high. The valid ball candidate was 4 x 7 pixels
with 21 dark pixels and 75% density; it was rejected only because its bottom
edge landed on the search ROI rejection boundary.

- Keep the search ROI top margin derived from the pipe height to reject the dark
  upper pipe boundary.
- Use a fixed 3-pixel bottom margin so the lower half of the ball remains inside
  the search ROI.
- Keep the existing edge, size, aspect-ratio, and density filters. Do not disable
  them globally, because pipe edges would become valid candidates.
- Draw only the derived physical pipe display box in the IDE preview. The
  padded detection ROI and narrower search ROI remain internal and are not
  shown as additional pipe boxes.

## Uneven Pipe Lighting Correction

Two live frames showed that the left bright pipe section needs thresholds of
about 91-108 while the full-width threshold was 115-126. A single histogram for
the whole pipe therefore merges the ball with the darker left background.

- Keep the current single-ROI detector as the fast path.
- If the fast path finds no accepted ball, split the usable bright pipe span
  into four horizontal tiles. Expand each tile by 6 pixels on both sides, clip
  it to the usable span, and compute an independent histogram threshold.
- Run fallback tile blob searches with `merge=False`; score every candidate with
  the existing size, density, aspect-ratio, edge, and tracking-distance rules.
- Exclude 4 pixels at both horizontal image ends. The black left end cap is
  outside the required detection range; the full bright pipe section remains
  searchable.
- Run tiled fallback only after the fast path misses. Once reacquired, the
  existing local tracking ROI becomes the fast path again.

## Pipe Display Box

The detection ROI intentionally contains background margin, so it must not be
drawn as if it were the physical pipe boundary. Derive a display-only box from
the detection ROI by trimming one third of its height from the top, 2 pixels
from the bottom, and 4 pixels from each side. Continue using the untrimmed ROI
for detection and re-locking.

## Calibration

`MM_PER_PIXEL` must be measured on the real pipe. The existing scripts use about `0.76 mm/pixel`, so the first implementation keeps that value as a tuning constant.

## Failure Handling

- Send `$L#` after the ball is not confidently detected.
- Keep the previous smoothed position briefly to stabilize short occlusions.
- Expand from tracking ROI back to full pipe ROI after lost frames.
- Re-lock pipe ROI after repeated losses.

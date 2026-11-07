# 1.2.0

- Higher frame rate on the radio (on an X20, Low now flies at 14-17 fps, High at 10-12):
  - Runs full screen, without the title bar.
  - Only the parts of the screen that change are redrawn; the flight data sits on bars above and below the scene and is redrawn every other frame.
  - Faster scenery drawing: trees are a round crown on a trunk, and Low no longer draws the mown strip's edge lines.
  - The physics steps run in one call (same flight behaviour, bit for bit).
- The helicopter is drawn where it is at the moment of the frame, so fast passes no longer stutter.
- The CPU stats show fps, wakeups per second and the mean wakeup and paint time.
- Thanks to b14ckyy for this work.

# 1.1.0

- New **Detail** setting, default **Low**, for a higher frame rate on the radio: flat hills and tree line, no mown strip, one-triangle trees, a two-triangle shadow and the low detail model until the helicopter is close. **High** is the previous look.

# 1.0.0

- Initial release of HeliSim.
- Low-poly scale helicopter with a tilting rotor disc, a dark belly and a ground shadow.
- Acro and self-level flight modes; 3D and normal collective (no expo on collective).
- Settings: cyclic and yaw rates, expo, head response, spool-up time, rotor direction, auto zoom, CPU stats.
- Flying field with hills, a tree line, trees and a helipad; the camera follows the helicopter from the pilot's position.
- Enter resets to the pad, Page opens the settings.

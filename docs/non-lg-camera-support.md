# Non-LG Android camera support — known limitations

flowpilot was originally tuned for the LG G8. As of the "Auto-detect camera
intrinsics to support non-LG Android phones" change (PR #1), the camera
intrinsics (focal length and principal point) are derived at runtime from the
bound camera's hardware characteristics on non-LG devices, instead of using the
hardcoded LG G8 values.

Detection reads:

- `LENS_INFO_AVAILABLE_FOCAL_LENGTHS` — focal length in mm
- `SENSOR_INFO_PHYSICAL_SIZE` — physical sensor size in mm

and computes the focal length in pixels as `focal_mm × frameWidth / sensorWidth_mm`,
placing the principal point at the image center. The detected focal length is
shown in **Settings → Device → Camera Focal Length (px)** for verification.

LG devices, and any device with a manual `camerainfo.big.txt` /
`camerainfo.medium.txt` override file, keep their existing known-good behavior.

## Known limitations

These are inherent limitations of estimating intrinsics from hardware metadata
rather than code defects. The calibration / digital-zoom path partly compensates,
but they can still yield off geometry on certain devices.

1. **Ultrawide / wide lenses.** If the selected back camera resolves to a wide
   or ultrawide lens, the auto-derived focal length can be short enough that
   `digital_zoom_apply` falls below `1.0`. The camera cannot zoom out past its
   minimum zoom (`1.0`), so the captured field of view ends up wider than the
   intrinsics handed to the model — producing systematically wrong lane/lead
   geometry.

2. **Logical multi-camera devices.** Many modern phones expose a single logical
   camera backing several physical lenses. `LENS_INFO_AVAILABLE_FOCAL_LENGTHS[0]`
   and `SENSOR_INFO_PHYSICAL_SIZE` may not describe the physical lens CameraX is
   actually streaming, so the computed intrinsics can be incorrect even though
   detection appears to succeed.

## If you hit calibration issues on a non-LG phone

1. Check the detected focal length in **Settings → Device → Camera Focal Length (px)**.
2. If it looks wrong, provide a manual override via `camerainfo.big.txt`
   (big/F3 model) or `camerainfo.medium.txt` (medium/F2 model) in the flowpilot
   root. The file contains five lines: `FocalX`, `FocalY`, `CenterX`, `CenterY`,
   `UseCameraID`. A manual override always takes precedence over auto-detection.

# Limelight pipeline backups

Exported pipeline files (`.vpr`) for each camera. These hold the settings that live on the Limelight itself
(exposure, resolution, AprilTag family, field map, crop). If a camera is replaced or reflashed, upload these
from its web page and it's back to how it was.

Name them `<hostname>-pipeline<N>.vpr`, e.g. `limelight-front-pipeline0.vpr`.

The camera's mount position and which pipeline runs are set in code (`Constants.VisionConstants.kCameras`), not here.

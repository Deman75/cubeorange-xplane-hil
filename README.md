# Cube Orange X-Plane HIL2 — experimental firmware build

Build-only repository for a local prototype. It contains no aircraft geometry,
aircraft files, photos, radio binding information, or calibrated vehicle parameters.

The workflow fetches official ArduPlane 4.4.2 at the SHA in SOURCE_MANIFEST.json,
then applies the ExternalAHRS, airspeed and experimental steering overlay. Steering
adds an opt-in STEER2SRV_RATE_P yaw-rate feedback gain, default zero (legacy behavior).
This is an experimental controller change, not a validated flight release. The donor is
[qgerman2/ardupilot-Simulink](https://github.com/qgerman2/ardupilot-Simulink).
The donor's custom Simulink controller is **not** included.

Open **Actions → Build Cube Orange HIL2 → Run workflow** on the default branch.
On success, download the `CubeOrange-HIL2-EXPERIMENTAL` artifact.
The output is a firmware `.apj`, build logs, and the source manifest.

This is a build attempt, not a validated flight release. Do not flash until the
build and sensor protocol checks have been reviewed. The workflow never accesses
or flashes a flight controller and never publishes a GitHub release.

The matching Windows plugin is built separately in the local project.
An upstream HITL binary is incompatible with the local HIL2 protocol.

Use a private repository unless you intentionally choose to publish these sources.
Upload the **contents** of this folder, including `.github`; do not upload the ZIP
as a single repository file, and do not upload the surrounding local project.

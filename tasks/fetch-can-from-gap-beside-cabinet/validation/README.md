# Feasibility records

`CURRENT_REPORT.md` describes the delivered deep-drawer scene. Current evidence:

- `drawer_scan.csv` and `drawer_scan.json`: corrected full-drawer supplemental grid, both yaws and all failed layouts.
- `drawer_initial_width_attempt.json`: superseded preliminary width-reporting attempt, retained with its failures.
- `drawer_gripper_dims.json`: native-can entry/contact aperture and measured URDF hand dimensions.
- `drawer_demo_check.json`: full READY-to-landing demonstration and +10 mm reach reserve.
- `drawer_validation.json`: final pytest and actual mutation outcomes, with top-level result status.
- `drawer_tools_manifest.json`: hashes of external measurement and validation tools.

All other files here are historical short-wall/scaled-can scans, retained without deleting the underlying evidence. They do not describe the current scene. Historical `REPORT.md` and JSON contain their original environment records; the current report identifies superseded assumptions and corrections.

Current CPU checks use the existing `yam_env` Python and real yam_arena, with python-fcl supplied from an isolated temporary dependency target. No GPU simulation or achieved score is certified. Root README gives current testing and GPU handoff commands. External preparation scripts live in the designated reference-tools directory rather than the submission folder.

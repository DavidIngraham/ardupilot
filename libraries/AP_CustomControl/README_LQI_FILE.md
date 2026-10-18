# LQI feedback gain files

AI-assisted implementation. This is an experimental gain-only interface for the
existing microAGU 16-state observer and three-integrator controller.

Set `CP3_FILE=1`, place `lqi-gains.bin` at the filesystem root, and reboot. Files
are loaded once in backend construction, never during the control loop. The gain
source is latched on engagement. Missing, corrupt or incompatible files prevent
LQI engagement when file mode is required. `CP3_FILE=0` uses compiled gains.

The fixed 280-byte format is generated and checked by the microAGU CLI pipeline:
`python -m microagu_sim.pipeline verify-gains FILE COMPILED_REFERENCE_HEADER`.
It contains magic/version, compiled-model CRC32, payload CRC32, 3x19 feedback
values and 3x3 anti-windup values (little-endian IEEE binary32). Finite values,
exact length and anti-windup consistency are checked before committing the table.
CRC32 is not authentication. The release pipeline records SHA-256 hashes.

`AP::FS()` provides the same reader for ChibiOS storage and WASM MEMFS. For WASM,
write the file in Emscripten `preRun` before firmware starts. The microAGU viewer
requires firmware acknowledgement before accepting startup with a gain file.
ChibiOS builds must explicitly enable `AP_PLANE_CUSTOMCONTROL_ENABLED` and
`AP_PLANE_CUSTOMCONTROL_LQI_ENABLED`; hardware defaults remain unchanged.

Validation: WASM ArduPlane build, MatekH743 ChibiOS build with both flags enabled,
Python-to-C++ decoder tests, valid-file EKF flight campaigns, corrupt-file rejection
and browser acknowledgement tests. No SD-card bench test or hardware flight has
been performed. The full study and commands are in microAGU's
`research/controller-pipeline-2026-10-18/RESULTS.md`.

A changed observer/state layout needs matching firmware, not just a new file.
Update the file version if controller semantics change incompatibly.

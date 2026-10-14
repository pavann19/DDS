# Abandoned camera publisher recovery

Parent: `5cedd461e2f177ef2a784a8d43e216fc09620e34`. Sandbox only; main remains `390f62300c1e7a030fe2717e1f90e9d6834cb523`.

Camera ownership expires after two seconds without an accepted frame. A fresh replacement publisher may then acquire the same epoch. Capture-time high-water remains global; sequence high-water is publisher-specific. Active-owner conflicts, duplicate captures, stale/future images, processing bounds and post-processing revalidation remain enforced. The two-second lease is longer than the unchanged 350 ms camera freshness limit, so a camera safety hold precedes ownership replacement.

Tests: 23 ingress and rig tests passed, including replay rejection during expired-owner takeover and rejection of the old publisher once a new owner is active.

Live browser evidence: original agent tab produced accepted frames, replacement tab opened, original tab closed. Replacement audit showed 195 accepted frames with latest status 200. No scenario restart was invoked. Eight-second telemetry capture retained epoch 1: 81 samples, 73 ready, seven recovering, one stale; speeds 47.52–49.88 km/h. Raw local evidence is output/camera-publisher-recovery.json.

Residual issue: replacement tab audit recorded a 764.7 ms maximum render gap, with visibility history containing only initial visible. This demonstrates that hidden-tab gating alone cannot explain all camera failures. Browser/UI render stalls still require isolation or a separate sensor renderer. Roadside-control recognition has not been validated across the complete route; no full smooth-driving acceptance is claimed.

Camera delivery changes are separately recorded in DDS_CAMERA_DELIVERY_FIX_2026-10-13.md. Keep the sandbox and main distinct. No push, PR, or deployment.

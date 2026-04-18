## Architecture notes

- `mowgli_mavros_bridge` is not an additional helper node. It is intended to be a second hardware backend, functionally replacing `mowgli_hardware` when selected by configuration.
- Backend selection is controlled by environment/configuration. When the backend is switched in `.env`, the system should behave as a full replacement, not as a partial overlay.
- Reviews must check whether the MAVROS backend preserves the externally expected ROS graph, topic names, remaps, message types, and semantic behavior currently provided by `mowgli_hardware`.
- Do not assume hardware behavior is finalized yet. Hardware-specific mappings may remain provisional until bench and Pixhawk validation are complete.
- Prefer coherent software-side fixes that improve compatibility and reduce regression risk without hard-coding assumptions that require final hardware validation.

## Review guidelines

- Treat `mowgli_mavros_bridge` as an alternative backend to `mowgli_hardware`, not as an extra node.
- Flag anything that would prevent a full backend swap through configuration.
- Compare bringup integration, remaps, published topics, subscribed topics, status/power semantics, and failure behavior against `mowgli_hardware`.
- Distinguish between:
  - software integration issues that should be fixed now
  - hardware-dependent behavior that must be left configurable or explicitly marked provisional
- Prefer minimal patches.
- Do not recommend merge-ready claims for behavior that still depends on unvalidated hardware wiring or Pixhawk feedback paths.
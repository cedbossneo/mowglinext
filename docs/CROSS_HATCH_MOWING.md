# Cross-hatch mowing

Settings → Mowing → **Cross-hatch mowing** saves `mow_cross_hatch` (default
`false`). Restart ROS2 after changing this setting. It works with both Auto
and a fixed Mow Angle. This rotates the stripe layout, not the blade motor.

The first coverage session uses the normal angle. The next uses that angle +90°,
then the next returns to normal. Fixed 25° therefore gives 25°, 115°, 25°.
Auto first resolves its normal angle for each interior polygon, then applies the
session's 0°/90° offset. It does not optimise the rotated angle back to the original.
All areas in a session share the offset; their Auto base angles can differ.
Boundary passes, obstacle boundaries and clearance settings are unchanged.

The orientation is saved with `coverage_resume.txt` in the existing maps volume.
Pauses, detours, low-battery recharge, replanning and ROS2/Pi restarts retain it.
A real `EndSession` advances the phase if coverage started, including a run later
abandoned and docked. Repeated session-end calls, failed plans, manual mowing and
sessions with cross-hatch disabled do not consume a phase. Resuming after STOP
continues the session. Clearing coverage progress alone preserves its orientation.
The first plan latches the setting for that session, so changing the toggle does
not rotate an interrupted session when ROS2 restarts.

At session end, one atomic snapshot clears the command and coverage cursors and
retains only phase metadata. This metadata cannot trigger automatic startup.
If persistence is disabled (`coverage_resume_path` empty), orientation survives
only in memory. Missing/deleted state starts with the normal angle again. Map or
base-angle edits can still change planned geometry; existing resume fingerprints
invalidate a cursor if its path changes.

## Auto-angle correction

For interior cells over 400 m², Auto uses the longest boundary segment to avoid
an expensive search on the Pi. `atan2` can return a negative heading; previously
that was mistaken for the Auto sentinel and ran the search anyway. The edge
heading is now normalised modulo 180° before testing the sentinel. Smaller cells
continue to search in 5° steps. The longest-segment heuristic itself is unchanged.

## Integration and verification

`PlanCoverage.action` adds `perpendicular` to its goal. The behavior tree and
coverage server must be rebuilt/upgraded together; ROS action type compatibility
changes. Firmware and its protocol are unchanged. No ROS .msg/.srv binding changes
are required by the GUI or firmware generators.

Tests exercise negative edge headings, both polygon windings, perpendicular
fixed/small-Auto/large-Auto geometry, identical boundary rings, actual BT action
goals, session reset, persisted restart/progress reset, failed/disabled sessions,
launch injection and the GUI toggle. This changes physical mowing routes and
requires the usual monitored commissioning before deployment. No mower is flashed
or operated by this PR.

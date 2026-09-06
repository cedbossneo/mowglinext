# Cross-hatch mowing

Settings → Mowing → **Cross-hatch mowing** saves `mow_cross_hatch` (default
`false`). Restart ROS2 after changing this setting. It works with both Auto
and a fixed Mow Angle. This rotates the stripe layout, not the blade motor.

Each area keeps its own orientation. Its first coverage session uses the normal
angle, its next uses that angle +90°, then it returns to normal. Fixed 25°
therefore gives 25°, 115°, 25°. Area-select mowing advances only that area;
an area that was not mowed keeps its next direction. A whole-lawn session can
therefore use different offsets in different areas.
Auto first resolves its normal angle for each interior polygon, then applies the
session's 0°/90° offset. It does not optimise the rotated angle back to the original.
Boundary passes, obstacle boundaries and clearance settings are unchanged.

The **Next stripe direction by area** panel shows each area's current and next
orientation. Change **Next mow** to choose its next orientation explicitly.
These choices save immediately in ROS2, without restarting it, and do not change
an active or paused plan. Fixed angles are shown in degrees; Auto is shown as
**Auto** or **Auto + 90°**, since its exact base angle is resolved per polygon
at planning time. Values use the running stack's base angle, not unsaved edits.
The cross-hatch enable toggle and base-angle setting still require save/restart.
Connection or persistence failures are displayed rather than reported as saved.

Each area's orientation and any next override are saved with `coverage_resume.txt` in the existing maps volume.
Pauses, detours, low-battery recharge, replanning and ROS2/Pi restarts retain it.
A real `EndSession` advances an area's phase if its coverage started, including a run later
abandoned and docked. Repeated session-end calls, failed plans, manual mowing and
sessions with cross-hatch disabled do not consume a phase. Resuming after STOP
continues the session. Clearing coverage progress alone preserves its orientation.
The first plan for an area latches the setting for that session, so changing the toggle does
not rotate an interrupted session when ROS2 restarts.

At session end, one atomic snapshot clears the command and coverage cursors and
retains only phase metadata. This metadata cannot trigger automatic startup.
If persistence is disabled (`coverage_resume_path` empty), orientation survives
only in memory. Missing/deleted state starts with the normal angle again. Map or
base-angle edits can still change planned geometry; existing resume fingerprints
invalidate a cursor if its path changes.
Phase history uses ROS area indices, like the existing coverage resume state.
Deleting/reordering areas can reassign those indices; review the next directions
after restructuring the map. The GUI map stream carries the original ROS IDs
alongside its filtered mowing-area list, so navigation areas cannot shift a
direction edit or area-select mowing command to another lawn.

## Auto-angle correction

For interior cells over 400 m², Auto uses the longest boundary segment to avoid
an expensive search on the Pi. `atan2` can return a negative heading; previously
that was mistaken for the Auto sentinel and ran the search anyway. The edge
heading is now normalised modulo 180° before testing the sentinel. Smaller cells
continue to search in 5° steps. The longest-segment heuristic itself is unchanged.

## Integration and verification

`PlanCoverage.action` adds `perpendicular` to its goal. The behavior tree and
coverage server must be rebuilt/upgraded together; ROS action type compatibility
changes. `CoverageOrientation.srv` adds read/set-next access on
`/behavior_tree_node/coverage_orientation`, exposed through the existing GUI
service proxy. Upgrade the GUI backend/frontend with ROS2 for the new controls
and area-ID metadata. Go service bindings and the virtual-map TypeScript type
are regenerated; the firmware protocol is unchanged.

Tests exercise negative edge headings, both polygon windings, perpendicular
fixed/small-Auto/large-Auto geometry, identical boundary rings, actual BT action
goals, selected-area isolation, mixed whole-lawn phases, session reset, persisted
restart/overrides, progress reset, failed/disabled sessions, service write failure,
launch injection and the GUI toggle. Browser tests cover the actual settings
controls, persistence across reload, unchanged active orientation, real area IDs
and unavailable ROS2. PR screenshots use mocked robot data. This changes physical mowing routes and
requires the usual monitored commissioning before deployment. No mower is flashed
or operated by this PR.

# Automatic blade direction

Settings → Mowing → **Automatic blade direction** saves `blade_auto_reverse`
in `mowgli_robot.yaml`. It defaults to `false`. Restart ROS2 after changing it.
With it disabled, a new session uses the existing direction 0.

When enabled, the behavior tree randomly requests direction 0 or 1 on the first
blade-enable command of a session. Coverage and manual mowing share the choice.
Repeated enable commands, pause/resume, obstacle detours, area changes and
temporary charging stops retain it. The existing `EndSession` action clears it
after a completed or abandoned session; a temporary `ClearCommand` does not.
Stopping manual mowing in place is a pause, so resuming retains the choice too.
Random selection can repeat the previous direction; this is not strict alternation.

The choice lives in ROS2 memory, not the settings file or coverage resume file.
A ROS2 restart selects again on the next enable, including when resuming saved
coverage. Resetting coverage progress alone does not change blade direction.
Both command paths log the requested direction. The unsigned RPM feedback does
not identify the physical rotation direction.

## Firmware and hardware requirement

This feature affects physical blade behavior. Enable it only with a blade
assembly suitable for both directions and firmware that implements the direction
request **and waits for the blade to stop before reversing**. Firmware remains
the sole blade safety authority; the host's dry-run gate and emergency handling
are unchanged. No host command bypasses firmware checks.

The upstream firmware at the base of this change (`2d45cab4`) ignores the blade
direction argument. This ROS2/GUI change alone does not add firmware reversal.
The separately maintained LFP firmware includes direction handling and a stopped
reversal guard; that firmware work is not included in this PR. Version numbers
alone are not a reliable capability check for custom builds.

## Validation before enabling on a mower

Software tests capture real BT service requests against a fake hardware service:
both command paths retain reverse through OFF/ON, and session reset sends no
command. The launch/config and GUI tests cover the default and toggle wiring.

Physical validation is still required: with blades removed, confirm both physical
directions, zero-speed reversal behavior, repeated enable commands and recovery
after a ROS2 restart. Confirm pause, emergency and dry-run inhibit still stop or
prevent blade operation. Perform the repository's monitored commissioning
procedure before normal mowing. No mower is started or updated by this PR.

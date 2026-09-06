# Automatic blade direction

Settings → Mowing → **Automatic blade direction** saves `blade_auto_reverse`
in `mowgli_robot.yaml`. It defaults to `false`. Restart ROS2 after changing it.
With it disabled, a new session defaults to direction 0; an explicit map-menu
choice still overrides it.

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

## Map blade controls

The map menu's **Blade forward**, **Blade backward** and **Blade off** actions
call `/behavior_tree_node/mower_control` through the GUI's `blade_control` API.
They share the session policy with coverage and manual mowing. An explicit
forward/backward choice replaces the random choice until `EndSession`, including
through pauses and temporary stops. Firmware performs the stopped reversal.

Blade off inhibits subsequent tree enables until forward/backward is selected
or the session ends. Choosing a direction cannot override the tree's OFF state:
while idle, transiting, docking or stopped by a guard, it selects the direction
for the next permitted enable. It does not start a mowing mode or move the mower.
These overrides are in memory and reset with ROS2, like the automatic choice.

OFF also sends a direct hardware request before latching the tree override, so
it can still reach hardware if the behavior tree is unavailable. A failed tree
request is reported as an error: a direct stop alone cannot guarantee the tree
will keep the blade off. There is no direct-hardware fallback for ON. The separate
manual-mode Stop action retains its existing high-level STOP and direct OFF path.
The service acknowledges acceptance of intent, not measured rotation or hardware
acknowledgement; the firmware remains responsible for physical motor control.

## Firmware and hardware requirement

This feature affects physical blade behavior. Enable it only with a blade
assembly suitable for both directions and firmware that implements the direction
request **and waits for the blade to stop before reversing**. Firmware remains
the sole blade safety authority; the host's dry-run gate and emergency handling
are unchanged. No host command bypasses firmware checks.

The upstream firmware at the base of this change (`2d45cab4`) ignores the blade
direction argument. This ROS2/GUI change alone does not add firmware reversal.
Standard 500 and 500B firmware support is proposed separately in
[PR #559](https://github.com/mowglinext/mowglinext/pull/559), including the stopped
reversal guard. That PR contains no LFP charging changes. Version numbers
alone are not a reliable capability check for custom builds.

## Validation before enabling on a mower

Software tests capture real BT service requests against a fake hardware service:
both command paths retain reverse through OFF/ON, and session reset sends no
command. Operator-service tests also cover forward/reverse overrides, repeated
manual ticks, coverage stop/resume, OFF inhibition and idle/guard precedence.
API tests verify routing, rejected requests and the independent OFF path. GUI
tests click the three actual map menu actions. Launch/config tests cover the
default and toggle wiring.

Physical validation is still required: with blades removed, confirm both physical
directions, zero-speed reversal behavior, repeated enable commands and recovery
after a ROS2 restart. Confirm pause, emergency and dry-run inhibit still stop or
prevent blade operation. Perform the repository's monitored commissioning
procedure before normal mowing. No mower is started or updated by this PR.

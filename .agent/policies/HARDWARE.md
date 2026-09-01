# HARDWARE.md — robot, firmware, and field evidence

Load this cumulative module for physical robot/field work; hardware, firmware,
kernel/driver/interface/topology; motor/actuator/sensor behaviour; physical
timing; reconnect/hotplug; serial, CAN, USB, RF, or other buses/transports; and
hardware bridge boundaries.

`CLAUDE.md` remains the authority for MowgliNext architecture, physical safety,
firmware/ROS2 authority, emergency/docking, hardware bridge, configuration, and
subsystem invariants. Consult it and linked references as required by CORE.
This module governs evidence/workflow and must not duplicate or replace those
project invariants.

## Hardware boundary and evidence

Before changes, record the exact evidence baseline as applicable: repository
and firmware commits, parent gitlink and submodule worktree `HEAD`, hardware
revision, robot/unit identity, firmware image/configuration, kernel/driver,
interface/topology, bus/port/radio setup, power/environment, test procedure, and
observation time. Hardware evidence is valid only for that exact baseline; do
not generalize it silently after any relevant component changes.

Trace authority across firmware, ROS2/host, hardware bridge, driver/kernel, bus,
and physical device. Establish owners of safety state and commands, lifecycle
and invalidation, clock/timing domains, reconnect/hotplug and source-incarnation
semantics, failure/degraded behaviour, and actuator/sensor authority. Never
replace an ownership/provenance defect with a timeout alone.

Do not claim physical safety, real-time timing, RF/bus integrity, actuator
response, sensor accuracy, reconnect/hotplug behaviour, or field fitness from
software-only evidence when hardware observation is required.

## Hardware-required stop condition

Use explicit states:

- `HARDWARE_REQUIRED`: remaining acceptance evidence intrinsically requires
  the stated hardware/field setup and has not yet been acquired.
- `HARDWARE_PENDING`: exact setup/procedure is identified and software-side
  prerequisites are complete, but execution/result remains pending.

When remaining uncertainty genuinely depends on hardware, stop expanding
software investigation. Record completed software evidence, why it cannot
answer the physical question, exact baseline/setup/procedure, observable
pass/fail criteria, safety prerequisites, owner/access dependency, and next
step. Do not substitute more repository searching.

## Safety and validation

Follow `CLAUDE.md` safety and authority boundaries. Do not energize motors,
actuators, cutting systems, docking power, or emergency-related outputs without
explicit authorization, a controlled setup, and applicable project procedure.
Prefer non-energized inspection, simulation, bench tests, then constrained field
validation when they can answer the contract.

Run firmware/build/generation/format commands as the normal project user, never
root. Validate focused software contracts before requesting hardware proof;
then record exact procedure, observations, logs/calibration where appropriate,
baseline, result, residual limitations, and invalidators. A test on one
device/revision/configuration proves only that baseline unless an explicit
compatibility argument is established.

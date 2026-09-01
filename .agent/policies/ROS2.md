# ROS2.md — ROS2-specific work

Load this cumulative module for ROS2 source/packages, C++/Python nodes,
interfaces, generated bindings, `msg`/`srv`, QoS, launch, Nav2/TF behaviour, or
`colcon` builds. Load other triggered modules and consult `CLAUDE.md` for every
mandatory architecture/safety case listed in the CORE.

## Formatting and non-root execution

Any change under `ros2/` touching C++ source/header files (`.cpp`, `.cc`,
`.hpp`, `.hh`) must be clang-format clean with repository style:

```bash
clang-format -i -style=file:ros2/.clang-format <files>
```

Before conclusion, prefer:

```bash
git clang-format --diff origin/main -- <files>
```

If `origin/main` is unavailable, at minimum:

```bash
clang-format --dry-run --Werror \
  -style=file:ros2/.clang-format <files>
```

Execute these commands, `colcon`, CMake, Ninja/Make, generators, and all other
artifact-writing commands as the normal project user—never root. In the
devcontainer that user is `ubuntu`: run commands directly when the effective
user is already `ubuntu`; if the wrapper runs as `root`, use
`runuser -u ubuntu -- <command>` to drop privileges. Repair/remove only affected
root-owned build artifacts while preserving unrelated state.

## ROS2 contract and validation

For node/launch/QoS changes, identify affected nodes, topics/services/actions,
QoS compatibility, parameters/configuration ownership, startup/shutdown and
reconnect lifecycle, clock domain, and integration consumers. For localization,
TF, Nav2, coverage, GNSS/RTK, hardware bridges, docking/emergency, configuration
ownership, or cross-layer behaviour, load `CLAUDE.md` and linked references
before behaviour changes.

For `.msg`/`.srv` or generated-interface changes:

- establish producer/consumer and backward-compatibility contracts first;
- regenerate with the project workflow as `ubuntu`;
- validate reproducibility and all affected language bindings/consumers;
- confirm C++/Python/Go/frontend equivalents where required;
- do not hand-edit generated outputs unless the owning workflow requires it.

Use focused ROS2 validation first, then affected packages/binaries, generated
equivalence, launch/integration tests, replay/simulation/hardware, and only then
broad workspace validation when justified. Record exact packages/commands,
distinguish baseline failures, verify formatting, run `git diff --check`, and
confirm gitlinks/submodules remain intended.

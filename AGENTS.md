## Install system model

The installation system is currently in a transitional state.

### Configuration flow (intended design)

The intended installation workflow is:

1. `install/config/*` contains editable template files
2. The installer will:
   - read and modify these templates
   - generate a `.env`
   - copy and adapt files into `docker/config/*`
3. The Docker stack is launched using the generated `docker/config/*` and `.env`

Important:
- `install/config/*` is NOT the runtime configuration
- `docker/config/*` is the runtime configuration used by Docker Compose

### Current state

- The template → runtime copy/adaptation is NOT fully implemented yet
- Some configuration is still done manually
- The system must still be installable in its current form

### Review rules for install system

When reviewing installation and Docker-related code:

- Validate only the **current nominal install path**
- Do NOT treat the following as blocking issues:
  - differences between `install/config/*` and `docker/config/*`
  - unused or partially integrated template files
- Do NOT assume template processing is complete

Instead, focus on:
- whether the installer can run to completion
- whether `.env` contains required variables
- whether `docker compose config` is valid
- whether bind mounts point to existing runtime files
- whether the selected backend can start correctly

---

## Optional / future features (do not flag as critical)

The following are intentionally incomplete and should not be treated as blocking bugs:

- Foxglove activation is not yet automated (manual setup currently required)
- VESC support is not yet implemented (placeholders exist)
- TF-Luna / range sensor support is not yet implemented (placeholders exist)

These components:
- may appear in Docker Compose fragments
- may define variables not yet used
- should only be flagged if they break the default installation path

---

## Backend Consistency Rules (Strict)

These are hard constraints and must not be violated.

- A service that is specific to the MAVROS backend must never default to `mowgli`.
- A service that is specific to the Mowgli backend must never default to `mavros`.
- Backend-specific services must not rely on implicit or fallback values for backend selection.
- If `HARDWARE_BACKEND` is used, it must reflect the actual role of the service.

### Invalid Patterns (must be flagged)

- `HARDWARE_BACKEND: ${HARDWARE_BACKEND:-mowgli}` inside a MAVROS service
- `HARDWARE_BACKEND: ${HARDWARE_BACKEND:-mavros}` inside a Mowgli service
- services whose behavior depends on backend but do not receive the variable
- entrypoints that enforce a backend but receive a conflicting value

### Expected Behavior

Switching `HARDWARE_BACKEND` in `.env` must:

- fully switch the active backend
- not require manual edits in Compose files
- not leave partially active components from another backend

### Source of Truth

- `.env` is the only authoritative source for backend selection
- Compose files must not redefine backend logic independently

## GPS Backend Contract

- GPS behavior depends on `HARDWARE_BACKEND`.
- When `HARDWARE_BACKEND=mavros`:
  - GPS is handled through Pixhawk/MAVROS
  - direct GPS drivers (e.g. ublox, unicore) must not be started
  - only the NTRIP / RTCM delivery path may be active if needed
- When `HARDWARE_BACKEND=mowgli`:
  - GPS is handled directly by the Mowgli stack
  - a GPS backend selection (e.g. ublox, unicore) is valid
- GPS backend selection must not be treated as global if `HARDWARE_BACKEND=mavros`.
- Reviews must flag any configuration where MAVROS mode still enables a direct GPS driver path.

- A service may receive `HARDWARE_BACKEND` only to disable itself or switch to the correct mode.
- In MAVROS mode, GPS-related services must not fall back to legacy direct-GPS behavior.
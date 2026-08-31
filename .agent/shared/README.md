# Shared agent handoffs

This directory is for short-lived handoff material that must be shared through
Git between contributors or agents.

Use it for compact unfinished-work state such as:

- cross-contributor handoffs;
- temporary remediation matrices;
- shared execution plans;
- resumable investigation state.

Do not use it for local session checkpoints. Those belong under
`.agent/checkpoints/` and remain ignored.

Do not use it as permanent project documentation. Validated reusable knowledge
must be promoted to:

- `docs/audits/`
- `docs/analysis/`
- `docs/architecture/`

Every shared handoff should identify its repository baseline, established
evidence, unresolved items, scope boundaries, and exact next step.

Remove or promote the handoff once its collaboration purpose is complete.
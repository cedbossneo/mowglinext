# Shared agent handoffs

This directory is for short-lived handoff material that must be shared through
Git between contributors or agents.

Use it for compact unfinished-work state such as:

- cross-contributor handoffs;
- temporary remediation matrices;
- shared execution plans;
- resumable investigation state;
- large audit state that another contributor or agent must continue before it is
  mature enough for permanent documentation.

Do not use it for local session checkpoints. Those belong under
`.agent/checkpoints/` and remain ignored.

Do not use it as permanent project documentation. Validated reusable knowledge
must be promoted to:

- `docs/audits/`
- `docs/analysis/`
- `docs/architecture/`

or an established project-specific versioned audit/analysis ledger when keeping
its current location is clearer than moving it.

Every shared handoff should identify, as applicable:

- repository baseline / commit;
- branch or upstream/base relationship;
- relevant submodule worktree commit and parent gitlink;
- established evidence;
- semantic or architectural decisions already settled;
- unresolved items;
- scope boundaries and "do not touch" areas;
- validation already completed;
- validation still required;
- exact next step.

When a handoff transfers audit, backlog, or remediation state:

- preserve the stable finding/item identifiers used by the authoritative ledger;
- preserve established project-specific identifiers such as `MGNSS-*` instead
  of renumbering them;
- use line numbers only as secondary baseline information, not as the primary
  identity of a finding or task;
- keep STATUS separate from SCOPE/TYPE when classifications are transferred;
- identify relevant durable evidence sources as `CURRENT`, `PARTIALLY_STALE`, or
  `SUPERSEDED` when freshness matters;
- for `PARTIALLY_STALE` evidence, state the known stale conclusion/section and
  the replacement evidence;
- preserve canonical duplicate and dependency relationships rather than
  creating a second incompatible classification graph;
- for cross-repository or submodule findings, preserve the parent `HEAD`, parent
  gitlink, submodule worktree `HEAD`, and relevant upstream baseline;
- reference the authoritative audit/ledger instead of copying the full ledger
  into the handoff;
- include only the minimum state needed for another contributor or agent to
  resume safely.

A shared handoff is not a second source of truth. If an authoritative durable
ledger or audit already exists, reference it and record only the temporary
execution state, unresolved delta, ownership transfer, and next action.

Keep shared handoffs concise. Do not copy conversation transcripts, large logs,
generated artifacts, secrets, credentials, unnecessary source content, or full
copies of durable audit ledgers.

Remove or promote the handoff once its collaboration purpose is complete. If its
findings become stable and reusable, move that knowledge into durable versioned
project documentation and retire the temporary handoff.
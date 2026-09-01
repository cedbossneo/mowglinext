# AUDIT.md — audits and structured backlogs

Load this cumulative module for audits, reviews, TODO/backlog reconciliation,
remediation matrices, structured findings, or durable audit ledgers. Load
`LONGTASK.md` too when its independent triggers apply.

## Analysis-first and evidence ledger

Default to analysis-only. Do not modify production code unless remediation is
explicitly authorized. Establish repository/branch/`HEAD`, base, relevant
gitlink/submodule baselines, scope, authoritative ledger, and exact behavioural
questions before broad evidence collection.

Give findings/items stable IDs before they become dependency or collaboration
targets; preserve established project IDs such as `MGNSS-*`. Line numbers and
Markdown positions are secondary, mutable baseline data. Preserve identity or
record an explicit canonical relationship when an item is renamed, moved,
deduplicated, or transferred between repositories/owners.

Keep these axes separate:

```text
STATUS: IMPLEMENTED | PARTIAL | OPEN | BLOCKED | SUPERSEDED | OBSOLETE | DUPLICATE
SCOPE: ROS2 | NAVIGATION | LOCALIZATION | GNSS | GUI | BACKEND | FRONTEND |
       FIRMWARE | HARDWARE_BRIDGE | CONFIGURATION | DEPLOYMENT | SUBMODULE |
       DOCUMENTATION | VALIDATION | DOWNSTREAM | <justified project scope>
VALIDATION: exact evidence/command/result or explicit pending requirement
```

When `HARDWARE.md` applies, `VALIDATION` may be `HARDWARE_REQUIRED` or
`HARDWARE_PENDING` as defined by that module. These validation states remain
orthogonal to `STATUS` and `SCOPE`; never interpret either value as a task
`STATUS`. Do not duplicate their detailed definitions here.

Scope never implies status. Use `PARTIAL` only when part of the same requested
contract exists, recording `Existing`, `Missing`, and `Completion criterion`.
Related infrastructure elsewhere is insufficient. Use `BLOCKED` only for a
real dependency preventing safe/meaningful progress, recording `Blocked by`
(stable ID or external dependency) and observable `Unblocks when`. Preferred
ordering alone remains `OPEN` with an ordering note.

## Conservation and duplicate graph

Before editing an authoritative backlog, freeze classification and verify:

```text
original item count = sum(all classified items)
original item count = remaining items + intentionally removed items
```

Every removed item retains original identity/text, status, scope, evidence,
and disposition in the audit ledger. Recount after cleanup and reconcile all
retained/removed items.

`DUPLICATE` is directional: each duplicate references exactly one canonical
non-duplicate item. Reject cycles (`A -> B`, `B -> A`), duplicate-to-duplicate
chains, missing targets, and incompatible dependency graphs. Validate numeric
conservation and canonical duplicate/dependency references before and after
edits.

## Freshness and contradiction handling

Classify durable evidence used by the audit as `CURRENT`, `PARTIALLY_STALE`, or
`SUPERSEDED`. For partially stale evidence, identify the bounded stale
conclusion and replacement. Never treat historical analysis as current without
comparison to current source/Git/tests/contracts.

If stale durable documentation is found, record the contradiction and repair it
only within scope or create an explicit follow-up so it is not reused silently.
For parent/submodule contradictions, record parent `HEAD`, parent gitlink,
submodule worktree `HEAD`, and authoritative upstream evidence.

## Audit freeze workflow

Prefer:

```text
baseline
 -> targeted evidence collection
 -> item classification
 -> classification freeze
 -> duplicate/dependency validation
 -> mechanical count validation
 -> contradiction review
 -> exact cleanup/remediation plan
 -> authorized edits
 -> post-edit conservation and validation
```

Do not edit the authoritative backlog while classifications are moving unless
an incremental live ledger is explicitly required. Keep the detailed ledger in
one authoritative versioned location once mature; use local checkpoints for
session state and `.agent/shared/` only for temporary collaboration deltas.

An audit closes only after applicable count, duplicate canonicalization,
dependency reference, freshness, contradiction, and post-cleanup checks pass or
are explicitly reported unresolved. Promote reusable validated findings to a
durable audit/analysis document with baseline and staleness conditions.

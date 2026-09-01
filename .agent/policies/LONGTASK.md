# LONGTASK.md — dependent and resumable work

Load this cumulative module for dependent multi-step work; work that must
survive interruption/compaction; materially expanding investigation;
architecture, lifecycle, ownership, freshness, provenance, or concurrency
reasoning; checkpoint/handoff work; collaboration; or multiple durable
findings. Low budget alone is not a trigger.

## Early checkpoint and evidence cache

Create a task-specific checkpoint under `.agent/checkpoints/` as early as
practical and before roughly one third of expected investigation/reasoning.
Update it before broad builds, suites, searches, migrations, refactors,
generated workflows, or other context-heavy operations, and after major
discoveries. Prefer a current partial checkpoint to unrecorded analysis.

Use names such as `<TOPIC>_RECON_CHECKPOINT.md` or
`PHASE_<X>_CHECKPOINT.md`. Keep it summary-first, compact, and operational—not a
conversation transcript. Record as applicable:

- repository, branch, `HEAD`, base/remotes, dirty state, and relevant gitlinks
  and submodule worktree commits;
- objective and authorized scope;
- established facts and exact files/symbols/evidence;
- architecture/data flow, authoritative producers/consumers, ownership,
  lifecycle/invalidation, clock domains, provenance/incarnation/reconnect, and
  compatibility conclusions;
- semantic decisions, rejected alternatives, and invalidation conditions;
- files inspected/expected to change and "do not touch" boundaries;
- regression matrix, commands plus `PASS`/`FAIL`, repository/file state at
  validation, and what invalidates results;
- unresolved questions, risks/limitations, and exact next action/command.

Explicitly separate:

```text
Established facts — DO NOT REDISCOVER
Semantic decisions — DO NOT RE-LITIGATE WITHOUT NEW EVIDENCE
Validation evidence — DO NOT RERUN UNLESS INVALIDATED
Open questions
Exact next step
```

Reference detailed ledgers, audits, issues, commits, or tests instead of copying
logs/diffs/source. For large audits, store baseline and counts once; keep only
contradictions, decisions, invalidators, unresolved state, and next action in
the checkpoint.

## Resume and freshness comparison

On resume, reload CORE and all active modules, read only task-overlapping
checkpoint/handoff state, then revalidate path, branch, `HEAD`, remotes, dirty
state, and relevant gitlinks/submodules. Compare changed files and dependencies
with the recorded evidence. Reuse still-current conclusions; investigate only
missing, ambiguous, stale, contradicted, or invalidated parts.

Compaction, time, model switch, new session, or loss of remembered reasoning is
not invalidation. Do not repeat archaeology or tests solely because a different
agent/model is continuing.

## Reconnaissance and semantic contract

Use progressive investigation:

1. Git/repository identity and exact target symbols/contracts;
2. relevant producer, consumer, configuration, and tests;
3. only the surrounding implementation required to establish dependencies;
4. neighbouring components only when the graph remains unclear;
5. repository-wide work only when targeted evidence is insufficient.

Before cross-layer edits, establish authoritative producer for each state,
ownership, lifecycle/invalidation, each timestamp/timeout clock domain,
`SET`/`OMIT`/`CLEAR` or equivalent semantics, source/incarnation/reconnect,
backward compatibility, and regressions proving stale state cannot regain
authority. Do not patch lifecycle/provenance defects with arbitrary timeout
alone.

Trace full data/control flow and freeze the smallest architecture satisfying
the invariants before production edits. Do not speculate in production code
when an owning API, exact test, focused reproduction, or small historical diff
can answer the question more cheaply.

## Validation ladder and execution economy

Use the narrowest applicable progression:

1. static inspection/exact contract check;
2. focused deterministic regression;
3. affected component/package tests;
4. directly affected builds;
5. generated or cross-language equivalence checks;
6. integration/launch/ROS2 tests;
7. replay, sanitizer, simulation, or exact-baseline hardware validation;
8. broad monorepo validation only for a justified integration/release gate.

Investigate early failures before broader runs. Select tests by dependency:
leaf -> focused unit; shared library -> affected consumers; public contract ->
all known consumers; interface/schema -> generation and bindings/consumers;
node/launch/QoS -> affected node/integration; parser/state machine/transport ->
regression plus replay/sanitizer; firmware -> firmware plus host contract.

Do not rerun identical broad suites for reassurance. Preserve concise command,
exit status, meaningful result, baseline, and invalidators. Retain enough failed
output to diagnose, but avoid repeatedly printing large successful logs/diffs.

## Collaboration and durable knowledge

For independent findings, record shared baseline and invariants once, assign
stable IDs, severity/evidence/scope/dependencies/status, independence, and
ordering before splitting work. Each recipient must know what is established,
validated, forbidden, baseline-specific, still open, and exactly next.

Use `.agent/shared/` only when another contributor must resume before knowledge
is mature; keep one authoritative ledger and a minimal execution delta. Promote
validated reusable architecture/data-flow, ownership/lifecycle, clock,
provenance, compatibility, defects, remediation, regression, limitation, and
dependency knowledge to `docs/analysis/`, `docs/audits/`, or
`docs/architecture/`. Include baseline, evidence, assumptions, unresolved
questions, status, and staleness conditions. Do not remove the local checkpoint
until promotion is reviewed/preserved.

## Resource discipline and closure

Treat the CORE budget policy as always active. Avoid reopening settled
decisions, low-value branches, repeated large reads, optional refactors, and
reasoning-tier churn. Reserve budget for review, targeted validation, scope
confirmation, evidence promotion/checkpoint update, and handoff.

Avoidable compute is also avoidable energy use, but never invent or claim
quantified energy or carbon savings without actual measurement. Do not list
every successful read-only command when a compact evidence summary suffices.

Stop when the authorized contract is satisfied. Classify and defer adjacent
findings unless required for present correctness/safety. A long task is complete
only when its diff, validation, knowledge persistence, repository status,
limitations, and exact follow-up state are accurate.

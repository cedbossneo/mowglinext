# MowgliNext — Agent Rules v1.3

Repository-local execution policy for automated or AI-assisted contributors.

This file is the **always-loaded CORE**. Detailed policies live under
`.agent/policies/` and are loaded only when their trigger applies. Modules are
cumulative: they supplement this CORE and never weaken it or each other.

The objective is validated progress per unit of resource. Never trade
correctness, project contracts, physical safety, or recoverability for token,
context, compute, or time savings.

---

## 1. Instruction and evidence roles

Use each layer for its intended role:

- `AGENTS.md` -> stable, always-on agent execution policy;
- `.agent/policies/*.md` -> conditional detailed execution policy;
- `CLAUDE.md` -> MowgliNext architecture, safety boundaries, subsystem
  invariants, historical decisions, known traps, "What NOT to Do" rules, and
  subsystem references;
- subsystem documentation -> detailed technical contracts;
- `.agent/checkpoints/` -> local current-task operational state, never versioned;
- `.agent/shared/` -> intentional short-lived contributor/agent handoff state;
- versioned audit/analysis/architecture documents -> durable shared knowledge;
- source code -> authoritative current implementation;
- tests -> executable behavioural evidence;
- Git -> exact repository and implementation state.

Do not duplicate detailed project invariants from `CLAUDE.md` into agent policy,
or large bodies of information between these layers.

If applicable sources conflict:

1. determine whether one is stale or refers to another repository state;
2. prefer current source, tests, Git state, and applicable architecture/safety
   contracts over stale descriptive documentation;
3. resolve contradictions involving a `CLAUDE.md` invariant explicitly;
4. if the conflict cannot be resolved safely, stop and report it;
5. never silently guess which source to ignore.

A context compaction, model switch, restart, or new session is **not new
evidence**.

---

## 2. Session startup and conditional policy loading

Before substantial work:

1. read this CORE completely;
2. inspect only checkpoints/handoffs whose scope may overlap the task;
3. establish repository path, branch, `HEAD`, remotes, dirty/untracked state,
   and relevant submodule worktree/gitlink state;
4. compare applicable checkpoint/handoff assumptions with actual state;
5. identify the task type, owning subsystem, and architectural/public contracts;
6. load every policy module whose trigger applies;
7. read `CLAUDE.md` and linked subsystem references when required below;
8. reuse established evidence unless a recorded dependency changed.

Do not begin with repository-wide rediscovery.

### Conditional modules

Load `.agent/policies/LONGTASK.md` when any becomes true:

- the task spans multiple dependent implementation/investigation steps;
- an applicable checkpoint or handoff exists;
- architecture, lifecycle, ownership, freshness, provenance, or concurrency
  reasoning is materially required;
- work must survive interruption, compaction, restart, or another session;
- investigation expands materially beyond the initially identified component;
- collaboration or durable multi-finding analysis is involved.

Load `.agent/policies/AUDIT.md` for audits, backlog/TODO reconciliation,
structured classifications, conservation accounting, duplicate/dependency
graphs, or durable audit ledgers.

Load `.agent/policies/MIGRATION.md` when consulting historical PRs, forks,
branches, commits, or patches, or performing semantic porting/adaptation.

Load `.agent/policies/ROS2.md` for ROS2 source, packages, interfaces, generated
bindings, `msg`/`srv`, QoS, launch, builds, or ROS2-specific validation.

Load `.agent/policies/HARDWARE.md` for robot/field hardware, firmware,
kernel/driver/interface/topology, physical timing, reconnect/hotplug, buses,
serial/CAN/USB/RF, motors/actuators, sensors, or physical proof boundaries.

Low budget alone does not trigger `LONGTASK.md`. Never invent a trigger based on
tool-call count. A small isolated mechanical task may remain CORE-only.

---

## 3. `CLAUDE.md` consultation

Consult `CLAUDE.md` before changing behaviour involving:

- physical robot safety;
- firmware / ROS2 authority boundaries;
- localization or TF ownership;
- navigation or Nav2;
- coverage planning or tracking;
- GNSS / RTK integration;
- hardware bridge behaviour;
- docking or emergency behaviour;
- robot configuration ownership;
- cross-layer architecture;
- an invariant or subsystem explicitly documented by `CLAUDE.md`;
- an area for which `CLAUDE.md` links a more specific reference.

Read the linked subsystem reference when relevant. A clearly isolated typo,
format-only change, or unrelated mechanical edit need not reload the complete
`CLAUDE.md`. When in doubt about architecture or safety impact, consult it.

---

## 4. Reread, compaction, and resume rules

Already-loaded rules need not be reread after every command. Reread relevant
material when entering another phase/subsystem/repository, the environment is
rebuilt, context is compacted, the session restarts, actual state diverges from
the checkpoint/handoff, evidence contradicts prior conclusions, or a newly
touched contract requires another reference/module.

After compaction, restart, or model switch:

1. reload this CORE and every module active for the task;
2. read only the applicable checkpoint/handoff;
3. verify path, branch, `HEAD`, remotes, dirty state, and relevant gitlinks and
   submodule worktree commits;
4. compare the baseline and dependencies with actual state;
5. invalidate only conclusions whose dependencies changed;
6. inspect only what resolves discrepancies;
7. resume from the recorded exact next step.

Do not reconstruct the task by rereading the monorepo or repeating established
archaeology/tests. Time, compaction, model change, or another agent session alone
does not invalidate evidence.

---

## 5. Evidence discipline

- Distinguish verified behaviour from assumptions and hypotheses.
- Prefer current implementation and deterministic tests over names/comments.
- Cite exact files, symbols, tests, commits, or observations when useful.
- Do not re-litigate a recorded semantic decision without relevant new evidence.
- Evidence remains valid until an involved file, affecting dependency,
  repository/submodule baseline, relevant environment, generated interface, or
  contradictory evidence changes.
- When uncertain, identify the changed dependency and revalidate only the
  narrowest affected layer.
- Classify reused durable descriptions as `CURRENT`, `PARTIALLY_STALE`, or
  `SUPERSEDED` when freshness matters. For `PARTIALLY_STALE`, identify the stale
  section/conclusion and replacement evidence.
- Do not rerun expensive successful validation unless relevant changes, an
  explicit release gate, or a later contradiction invalidates it.

---

## 6. Agent-state persistence boundary

Agent state has exactly three classes:

1. `.agent/checkpoints/` -> ignored local operational memory. Keep compact
   baseline, established facts, decisions, validation/invalidators, open
   questions, dirty state, and exact next step. Never stage/commit it or use
   `/tmp` for state expected to survive compaction/restart.
2. `.agent/shared/` -> intentionally versionable, concise, short-lived handoff
   state. Record baseline, established versus unresolved facts, "do not touch"
   scope, validation, and exact next step. Exclude secrets, transcripts, large
   logs, generated artifacts, and duplicate ledgers; review before staging and
   remove/promote when its purpose ends.
3. `docs/analysis/`, `docs/audits/`, `docs/architecture/` -> durable validated
   reusable knowledge. Record baseline, scope, evidence, status, limitations,
   unresolved questions, and staleness conditions as applicable.

Reference detailed durable evidence instead of copying it. Do not delete a
checkpoint until promoted knowledge is safely preserved. Do not force future
contributors to repeat validated expensive work.

Repository ignore rules must preserve:

```gitignore
.agent/*
!.agent/policies/
!.agent/policies/**
!.agent/shared/
!.agent/shared/**
```

---

## 7. Non-root execution

Never build as `root`. Builds, tests, formatters, package managers, generators,
and commands that may modify repository artifacts, dependencies, caches, or
generated sources must run as the normal project user. This includes at least:

- `pio` / `platformio`;
- `colcon`, `cmake`, `make`, `ninja`;
- `npm`, `yarn`, `pnpm`, `npx`;
- `go generate`, Python package tooling, formatters, and generated workflows.

In the devcontainer, the normal project user is `ubuntu`. If the effective
current user is already `ubuntu`, run the command directly; never use `runuser`
by convention in that case. If the wrapper or session runs as `root`, explicitly
drop privileges with `runuser -u ubuntu -- <command>`.

Strictly read-only inspection may run as root when necessary. Repair ownership
or remove only the affected root-owned artifacts before continuing; preserve
all unrelated user state.

---

## 8. Scope control and investigation freeze

Requests framed as analysis, audit, review, investigation, reconnaissance, or
compatibility study are analysis-only unless implementation is explicit.

Before editing identify requested behaviour, owner, affected contracts and
consumers, acceptance evidence, and out-of-scope neighbours. Complete one
coherent finding/subsystem/phase before another. Do not opportunistically fix
unrelated defects or expand scope because adjacent code is improvable; record
such findings separately unless they block current correctness/safety.

Before substantial implementation, checkpoint the established contract and
authorized plan. Once implementation starts, broad investigation is frozen
unless implementation exposes a contradiction, deterministic evidence disproves
an assumption, required information is genuinely missing, or repository state
invalidates evidence. Interesting neighbouring work is insufficient.

If correctness requires an explicitly read-only/out-of-scope component, stop
and explain before modifying it. Never weaken an invariant, compatibility or
safety boundary, or test expectation to fit scope. Stop expanding when the
authorized contract is satisfied.

---

## 9. Budget and resource policy

Budget policy is always active and does **not** require `LONGTASK.md`.

Start with exact symbols/contracts and focused contexts; widen only when
insufficient. Batch independent read-only checks, avoid duplicate searches and
large repeated output, and use the least costly reliable reasoning tier. Protect
budget for diff review, validation, scope review, checkpoint, and handoff.

### Low-budget mode: below 20%

- stop broad exploration and optional refactoring/cleanup;
- do not open unrelated findings or subsystems;
- preserve semantic decisions and validation evidence promptly;
- prefer the narrowest useful validation;
- finish only the smallest safe operation already in progress;
- leave a precise next step.

Do not load `LONGTASK.md` solely because budget fell below 20%.

### Critical hard stop: at or below 10%

At `<=10%`, or clearly equivalent critical capacity, recoverability has absolute
priority. There is no "small/atomic enough" exception.

Do **not** start any new modification, failed-patch retry, edit, test, validation,
investigation, implementation substep, documentation, finding, subtask, or
subsystem.

Only:

1. allow an already-running command to finish;
2. inspect minimum Git state required for recovery;
3. consolidate already-established facts into the checkpoint;
4. record validation already acquired and still required;
5. record the exact next step;
6. stop.

The emergency checkpoint is limited to:

```text
HEAD / dirty state:
Established current state:
Exact next step:
```

Consolidation must not derive or verify new facts. Never claim completion merely
because the session is ending or consume remaining capacity because it exists.

---

## 10. Validation and completion

Validate from the cheapest sufficient layer upward. Always:

- start with focused validation for the changed contract;
- distinguish pre-existing failures from regressions;
- investigate a relevant early failure before widening validation;
- verify touched-language formatting/syntax;
- run `git diff --check`;
- preserve unrelated files and intended submodule/gitlink state.

For confirmed defects, define failure and acceptance criteria before production
changes and, when practical, add/identify a deterministic regression first.
Never weaken a test merely because a suspect implementation fails it; determine
whether implementation or expectation violates the contract.

Never claim a finding fixed or work complete while required validation is
failed, blocked, running, or unknown. Completion requires, as applicable:
requested behaviour; deterministic acceptance/failure-path evidence; affected
tests; reproducible generated bindings; cross-language equivalence; formatting;
clean diff check; expected submodules; untouched unrelated state; durable
knowledge/audit updates; explicit limitations and open findings.

---

## 11. Git safety

Never commit or push unless explicitly authorized.

Do not modify/discard unrelated user work, delete unrelated untracked files,
mix generated/environment noise into the patch, rewrite history, force-push,
destructively reset, silently update submodules/gitlinks, or stage checkpoints.

Before broad changes verify repository identity and baseline. Before concluding
substantial work, revalidate branch, `HEAD`, remotes, dirty state, and both
relevant submodule worktree `HEAD` and parent gitlink. If actual state differs
from an explicitly required baseline, stop before production changes and report.

Stage `.agent/shared/` only for intentional exchange after reviewing secrets,
stale noise, logs, generated content, transcripts, and local-only state. Prefer
durable documentation when knowledge is mature.

---

## 12. Final handoff

Report concisely, as applicable:

- files changed and why;
- tests/validation and important evidence;
- failures, limitations, and remaining findings;
- repository, branch, `HEAD`, and relevant submodule/gitlink state;
- local/shared agent-state disposition;
- commit status and push status.

Never omit changes performed. If incomplete, also report what remains and why,
latest checkpoint, shared handoff status, pending/blocked validation, and exact
next step. Never describe work as complete while required validation is running,
blocked, or unknown.

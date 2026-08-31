# AGENTS.md

Repository-local working rules for code agents.


## How agents must use this file

These rules apply to every automated or AI-assisted contributor, regardless of
model, vendor, agent framework, or reasoning tier.

`AGENTS.md` defines **how agents must work in this repository**:
investigation discipline, checkpoints, evidence handling, resource usage,
validation, Git safety, scope control, and handoff requirements.

`CLAUDE.md` is a **project architecture and maintenance reference**. It contains
high-value knowledge about MowgliNext's current architecture, safety boundaries,
subsystem invariants, historical design decisions, known traps, and
"What NOT to Do" rules.

Do not duplicate the detailed architecture content of `CLAUDE.md` into
`AGENTS.md`.

### At the beginning of every new agent session

Before substantial work:

1. read this `AGENTS.md` completely;
2. inspect the latest task checkpoint under `.agent/checkpoints/`, when one
   exists;
3. inspect any relevant shared handoff under `.agent/shared/`, when one exists;
4. verify checkpoint or handoff assumptions against the actual repository state;
5. establish repository path, branch, `HEAD`, remotes, dirty state, and
   relevant submodule/gitlink state;
6. determine which subsystem or architectural contracts the task touches;
7. read `CLAUDE.md` when the task depends on project architecture, safety
   boundaries, subsystem invariants, historical design decisions, or one of
   the areas documented there;
8. read the subsystem-specific reference linked by `CLAUDE.md` when relevant;
9. do not repeat investigation already recorded as established evidence unless
   repository state or new evidence invalidates it.

### When `CLAUDE.md` must be consulted

Consult `CLAUDE.md` before modifying behaviour involving, among other things:

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
- an invariant or subsystem explicitly documented in `CLAUDE.md`;
- an area for which `CLAUDE.md` points to a more specific reference document.

For trivial or isolated work that does not depend on those contracts, such as
a clearly scoped documentation typo, formatting-only change, or unrelated
mechanical edit, rereading the complete `CLAUDE.md` is not required.

When in doubt about whether a change can affect an architectural or safety
contract, consult `CLAUDE.md`.

### During an uninterrupted task

This `AGENTS.md`, `CLAUDE.md`, and already-read reference documents do not need
to be reread in full after every command.

Reread the relevant material when:

- entering a different work phase;
- crossing into another subsystem;
- changing repository or submodule;
- context was compacted;
- the devcontainer or execution environment was rebuilt;
- the agent session was restarted;
- repository state differs from the checkpoint or shared handoff;
- a contradiction with established evidence appears;
- new work touches an architectural or safety contract not previously loaded.

After context compaction or session restart, prefer:

```text
AGENTS.md
    -> latest local checkpoint and/or relevant shared handoff
    -> actual Git state
    -> CLAUDE.md / relevant subsystem reference when needed
    -> targeted verification
    -> resume work
```

instead of:

```text
repository-wide rediscovery
    -> historical archaeology
    -> repeated tests
    -> reconstructed reasoning
    -> eventual continuation
```

A context compaction, model switch, or new agent session is not new evidence.

### Instruction roles

When several repository instruction sources apply, interpret them by role:

- `AGENTS.md` -> agent execution policy;
- `CLAUDE.md` -> MowgliNext architecture, safety, and maintenance knowledge;
- subsystem documentation -> detailed technical contracts;
- `.agent/checkpoints/` -> local current-task state and already-established
  evidence;
- `.agent/shared/` -> intentionally shared, short-lived contributor/agent
  handoff state;
- versioned audit/analysis/architecture documents -> durable shared knowledge;
- source code and tests -> authoritative current implementation and executable
  evidence.

If two applicable instructions appear to conflict, do not guess which one to
ignore. Determine whether the conflict is caused by stale documentation or a
real architectural contradiction. If it cannot be resolved safely from the
repository evidence, stop and report it.

---

## ROS2 formatting

- Any change under `ros2/` that touches C++ source or headers (`.cpp`, `.cc`, `.hpp`, `.hh`) must be `clang-format` clean before finishing.

- Use the repository style file:

  ```bash
  clang-format -i -style=file:ros2/.clang-format <touched-files>
  ```

- Before concluding work on `ros2/` files, verify formatting explicitly. Preferred check:

  ```bash
  git clang-format --diff origin/main -- <touched-files>
  ```

- If `origin/main` is unavailable locally, at minimum run:

  ```bash
  clang-format --dry-run --Werror -style=file:ros2/.clang-format <touched-files>
  ```

---

## Non-root builds and tools

- Never build as `root`.

- Never run commands that may create or modify repository artifacts as `root`, including:
  - `pio`
  - `platformio`
  - `colcon`
  - `cmake`
  - `make`
  - `ninja`
  - `npm`
  - `yarn`
  - `pnpm`
  - `npx`
  - `go generate`
  - formatters
  - generated-file workflows

- Always use the normal project user so the repository does not accumulate root-owned files.

- If a previous run created root-owned artifacts, fix ownership or remove those artifacts before continuing.

- In the devcontainer, prefer:

  ```bash
  runuser -u ubuntu -- <command>
  ```

  or an equivalent shell running as the normal project user.

- Repository inspection commands may run as `root` when necessary, but any command capable of creating or modifying build artifacts, generated sources, dependency trees, caches, formatted files, or repository contents must run as the normal project user.

---

## Analysis and long-running task discipline

For any substantial analysis, audit, migration, architecture review, debugging session, or multi-step implementation, follow the rules below.

### Evidence discipline

For audits, debugging, architecture analysis, compatibility work, and semantic migrations:

- Distinguish verified source behavior from assumptions and hypotheses.
- Cite exact files, symbols, tests, commits, or runtime observations when useful.
- Do not infer behavior from names or comments when implementation can be inspected.
- Do not claim a historical change is still applicable without comparing it to the current architecture.
- Treat old PRs, forks, patches, and historical commits as behavioral evidence, not automatically mergeable code.
- When performing a semantic migration, explicitly classify historical changes as:
  - `PORT`
  - `ADAPT`
  - `ALREADY PRESENT`
  - `SUPERSEDED`
  - `REJECT`

### Agent state persistence policy

Agent working state has three distinct persistence levels. Do not mix their
roles.

#### 1. Local checkpoints

Store temporary task checkpoints under:

```text
.agent/checkpoints/
```

Examples:

```text
.agent/checkpoints/PHASE_G_RECON_CHECKPOINT.md
.agent/checkpoints/NAV2_AUDIT_CHECKPOINT.md
.agent/checkpoints/BLUEOS_MIGRATION_CHECKPOINT.md
```

Local checkpoints are operational memory for one contributor/agent and must
remain ignored by Git.

They exist primarily to survive:

- context compaction;
- model changes;
- agent restarts;
- interrupted sessions;
- long-running investigations;
- devcontainer restarts when the repository itself persists.

Do not use `/tmp` for checkpoints that are expected to survive those events.

Prefer a repository-local checkpoint over information preserved only in
conversation context.

Local checkpoints may contain transient details such as the current dirty
worktree, exact next command, partially completed validation, or unresolved
session-specific questions. Those details are useful for resumption but should
not normally become shared project history.

#### 2. Shared agent/contributor handoffs

Use:

```text
.agent/shared/
```

only when short-lived working knowledge genuinely needs to be exchanged between
contributors or agent sessions through Git.

Files under `.agent/shared/` may be committed intentionally.

Appropriate uses include:

- an unfinished cross-contributor handoff;
- an investigation state another contributor must resume;
- a temporary remediation matrix;
- a compact shared execution plan;
- a handoff needed while two contributors work on different parts of one audit.

Shared handoffs must:

- be concise;
- identify the repository baseline/commit they refer to;
- state what is established versus unresolved;
- state what must not be changed;
- include the exact next step when continuation is expected;
- avoid secrets, credentials, large logs, generated artifacts, and conversation
  transcripts;
- be removed or promoted when their short-lived purpose is complete.

Do not use `.agent/shared/` as permanent project documentation.

#### 3. Durable shared knowledge

Validated knowledge that will remain useful after the current task belongs in
versioned project documentation, preferably:

```text
docs/analysis/
docs/audits/
docs/architecture/
```

The intended lifecycle is:

```text
local investigation
    -> .agent/checkpoints/
    -> shared work required?
         -> no: remain local while active
         -> yes: .agent/shared/
    -> validated and reusable
    -> docs/analysis/ | docs/audits/ | docs/architecture/
```

Do not force another contributor or agent to repeat expensive validated
analysis merely because the original work happened in another session.

At the same time, do not preserve transient session noise as permanent project
documentation.

The repository `.gitignore` is expected to keep local `.agent` state ignored
while allowing intentionally shared handoffs, conceptually:

```gitignore
.agent/*
!.agent/shared/
!.agent/shared/**
```

If repository ignore rules do not match this policy, fix or report the mismatch
before relying on shared handoffs.

### Checkpoint compression and evidence cache

A checkpoint is a compact operational state, not a conversation transcript.

Record conclusions and evidence, not the full reasoning process.

Prefer:

- exact commit SHAs;
- exact file paths and important symbols;
- concise semantic contracts;
- authoritative producer/consumer relationships;
- ownership, invalidation, source-incarnation, and clock-domain conclusions;
- test command plus `PASS` / `FAIL`;
- the repository `HEAD` and relevant file state when expensive validation ran;
- rejected approach plus a one-line reason;
- exact next action or command.

Do not copy large logs, diffs, source files, PR discussions, or narrative
analysis into checkpoints.

When detailed evidence already exists in a versioned audit, architecture
document, test, commit, issue, or PR, reference it rather than duplicating it.

For expensive conclusions, use a compact structure such as:

```text
Decision:
Evidence:
Rejected alternatives:
What would invalidate this decision:
```

For expensive validation, record:

```text
Command:
Result:
Repository HEAD:
Relevant files/state:
What would invalidate this result:
```

The checkpoint should explicitly separate:

```text
Established facts — DO NOT REDISCOVER
Semantic decisions — DO NOT RE-LITIGATE WITHOUT NEW EVIDENCE
Validation evidence — DO NOT RERUN UNLESS INVALIDATED
Open questions
Exact next step
```

A checkpoint should make resumption cheap, not become another large document
that future agents must reprocess.

For long audits or backlog reviews, keep checkpoints summary-first:

- store the repository, branch, `HEAD`, base, and relevant submodule/gitlink
  baseline once;
- store classification counts and the current audit/remediation phase;
- store only contradictions, decisions, invalidators, unresolved questions, and
  the next action needed to resume safely;
- keep line-by-line item evidence in a separate local ledger or durable audit
  rather than duplicating it into the checkpoint;
- do not enumerate every successful read-only command when a compact preflight
  summary preserves the same recovery information.

A detailed ledger may be large when exhaustive traceability is required. The
checkpoint should reference that ledger instead of becoming a second copy of it.

Ensure `.agent/checkpoints/` and other local `.agent` working state remain
ignored by Git. Only `.agent/shared/` is eligible for intentional versioning,
and only under the shared-handoff rules above.

### Durable shared knowledge

When an analysis produces validated knowledge that can help future contributors or agents, promote the confirmed findings into versioned project documentation.

Preferred locations include:

```text
docs/analysis/
docs/audits/
docs/architecture/
```

Examples:

```text
docs/analysis/REPOSITORY_ARCHITECTURE.md
docs/audits/GNSS_DOWNSTREAM_AUDIT.md
docs/architecture/GNSS_CORRECTION_LIFECYCLE.md
```

Useful durable information includes:

- architecture and data-flow maps
- authoritative producers and consumers
- ownership and lifecycle contracts
- source / incarnation / reconnect semantics
- clock-domain rules
- public interface contracts
- compatibility constraints
- historical migration classifications
- confirmed defects and their evidence
- remediation decisions
- regression matrices
- residual limitations
- unresolved findings
- dependency and ordering constraints
- work that can be safely split among contributors

Shared knowledge records should preserve expensive analysis that would otherwise need to be rediscovered.

Do not force future contributors or agents to repeat repository-wide analysis that has already been validated and can be preserved accurately.

### Promoting analysis to shared knowledge

Before closing a substantial audit, architecture study, repository-wide analysis, or cross-layer investigation, determine whether its findings are useful beyond the current task.

If they are, preserve them in a versioned shared document rather than leaving the only durable copy in a temporary checkpoint or conversation.

A shared analysis document should record, as applicable:

- repository baseline / commit used for the analysis
- branch or relevant source baseline
- relevant submodule commits
- analysis scope
- date or remediation phase when useful
- verified facts
- assumptions and hypotheses, clearly marked
- unresolved questions
- source files, symbols, tests, PRs, or commits supporting important conclusions
- architectural invariants
- current status of findings
- remediation state
- regression coverage
- residual limitations
- conditions under which the analysis may become stale

Do not present historical analysis as current truth without revalidating it against changed repository state.

When a temporary checkpoint has matured into useful shared knowledge:

1. Keep only confirmed or clearly classified information.
2. Remove transient commands or session-specific noise that has no lasting value.
3. Preserve the evidence needed to revalidate important conclusions.
4. Record the repository baseline against which the conclusions were established.
5. Move or summarize the useful content into the appropriate versioned documentation path.
6. Do not delete the temporary checkpoint until the durable record has been reviewed or safely preserved.

### Early durable checkpoint

Create a durable checkpoint as early as practical when a task is likely to consume significant context, reasoning budget, execution time, or multiple agent sessions.

Do not wait until the end of the task.

The checkpoint should contain, as applicable:

- repository / branch / HEAD preflight
- confirmed scope
- current root cause or understanding
- architecture / data-flow map
- ownership and lifecycle conclusions
- relevant clock domains
- source / incarnation / reconnect semantics
- files inspected
- files expected to change
- tests already run and their results
- unresolved questions
- risks and residual limitations
- exact next implementation steps
- exact next commands when useful

Update the checkpoint incrementally after major discoveries.

### Checkpoint timing guarantees

For substantial tasks, the first checkpoint must be created before approximately one third of the expected investigation or reasoning effort has been consumed.

Before starting any operation likely to consume significant time or context, update the checkpoint first. Examples include:

- broad builds
- large test suites
- repository-wide searches
- migrations
- cross-layer implementations
- large refactors
- expensive generated-file workflows

If context compaction, model-budget exhaustion, service interruption, or session termination appears possible, preserving the checkpoint takes priority over additional investigation.

A partially complete but current checkpoint is preferable to a more complete analysis that exists only in conversation context.

### Resume from checkpoint

When a task-specific checkpoint already exists:

1. Read it before repeating repository exploration.
2. Revalidate branch, HEAD, worktree, remotes, and relevant submodules.
3. Verify that checkpoint assumptions still match the current source.
4. Reuse confirmed findings instead of rediscovering them.
5. Investigate only information that is missing, ambiguous, potentially stale, or invalidated by repository changes.

Do not blindly trust a checkpoint after repository state has changed.

Do not repeat expensive analysis solely because the current agent did not personally perform the original investigation.

When durable shared analysis already exists, read it before starting a new broad investigation and revalidate only the parts that may have changed.

### Budget-aware behavior

When model or context budget appears constrained:

1. Preserve confirmed findings before continuing exploration.
2. Stop opening low-value investigation branches.
3. Update the checkpoint before attempting risky or broad changes.
4. Prefer a complete implementation plan over a partial implementation.
5. Do not leave production code half-implemented solely to consume remaining budget.

If remaining capacity becomes critically low, stop exploratory work and consolidate the current state into the checkpoint.

### Reconnaissance before large changes

For large or cross-layer fixes:

- Trace the complete data/control flow before editing.
- Identify authoritative owners of state.
- Identify invalidation and lifecycle boundaries.
- Distinguish observed facts from assumptions.
- Build a focused regression matrix before implementation.
- Prefer the smallest architecture that satisfies the required invariants.

Do not begin broad production edits while the ownership or lifecycle model is still unclear.

Before editing a cross-layer system, establish:

- authoritative producer for each state
- ownership boundaries
- lifecycle and invalidation rules
- clock domain for each timestamp or timeout
- `SET` / `OMIT` / `CLEAR`, or equivalent update semantics
- source / incarnation / reconnect behavior
- backward-compatibility requirements
- regression cases proving stale state cannot regain authority

Do not solve lifecycle or provenance defects with an arbitrary timeout alone unless ownership and invalidation semantics have also been established.

### Checkpoint naming

Use a task-specific checkpoint name under `.agent/checkpoints/`, for example:

```text
.agent/checkpoints/PHASE_<X>_CHECKPOINT.md
.agent/checkpoints/<TOPIC>_RECON_CHECKPOINT.md
.agent/checkpoints/<TOPIC>_AUDIT_CHECKPOINT.md
```

Temporary checkpoint files are working artifacts unless the task explicitly requires them to become permanent project documentation.

Do not commit temporary checkpoints by default.

Promote validated reusable knowledge into `docs/analysis/`, `docs/audits/`, or `docs/architecture/` when appropriate.

### Analysis-first default

When the user asks for an:

- analysis
- audit
- review
- investigation
- reconnaissance
- compatibility study
- architecture study

or similar exploratory task, default to analysis-only.

Do not modify production code unless the user explicitly asks for implementation or the task clearly includes remediation.

For large analyses, create the first checkpoint before approximately one third of the expected investigation effort has been consumed.

### Resource, reasoning, and context discipline

Reasoning budget, context window, execution time, and tool calls are finite
engineering resources.

Use them deliberately.

The goal is not to minimize reasoning at the expense of correctness. The goal
is to avoid spending expensive reasoning on information that is already known,
already verified, irrelevant to the authorized scope, or recoverable cheaply
from repository state.

This policy intentionally reduces redundant reasoning, repository exploration,
tool calls, broad test execution, and repeated reconstruction of validated
knowledge. The primary goals are faster engineering iteration, lower compute
use, and better use of limited agent/model resources without reducing
validation quality.

Avoidable compute also means avoidable energy use. Resource efficiency is
therefore an engineering and environmental benefit, but do not invent or claim
quantified energy/carbon savings unless they have actually been measured.

#### Progressive investigation

Start with the smallest amount of exploration capable of answering the current
question.

Prefer this progression:

1. inspect repository identity, branch, `HEAD`, current diff, and relevant
   submodule/gitlink state;
2. search for the exact symbols, files, tests, contracts, topics, parameters,
   or interfaces involved;
3. read only the relevant surrounding implementation;
4. identify authoritative producers, consumers, ownership, lifecycle, and
   invalidation boundaries;
5. inspect neighbouring components only when the dependency requires it;
6. expand to repository-wide exploration only when targeted evidence is
   insufficient.

Do not begin a task by reading large portions of the monorepo "just in case".

Prefer exact-symbol search over full-file reading when the symbol or contract is
known.

Prefer focused line ranges over repeatedly reopening large files.

Batch independent read-only checks when practical.

Do not perform the same expensive search through multiple tools unless the
first result is incomplete, contradictory, or unsuitable.

#### Reuse established evidence

Do not repeatedly rediscover facts that have already been verified during the
current task.

Once a fact is established and recorded in the checkpoint or durable shared
analysis, treat it as an input unless later repository changes could invalidate
it.

When a versioned audit, analysis, architecture document, roadmap, `CLAUDE.md`
reference, or other repository document is reused as an evidence cache, classify
its freshness when that matters to the task:

```text
CURRENT
PARTIALLY_STALE
SUPERSEDED
```

Use these meanings:

- `CURRENT`: no known contradiction affects the conclusions being reused;
- `PARTIALLY_STALE`: some sections or conclusions are known to be outdated, but
  explicitly identified unaffected conclusions may still be reused;
- `SUPERSEDED`: the source must not be used as current authority except for
  historical context.

For a `PARTIALLY_STALE` source, record the known stale section or conclusion and
its replacement evidence. Do not discard the entire source when only a bounded
part is stale, and do not trust an entire document merely because it was once a
validated audit.

Source code, tests, current Git state, current submodule/gitlink state, and the
applicable architecture/safety contract remain authoritative when they
contradict stale descriptive documentation. If a contradiction touches a
`CLAUDE.md` safety or architecture invariant, resolve it explicitly rather than
silently overriding the reference.

Examples include:

- repository identity and baseline;
- branch and base commit;
- architecture ownership;
- authoritative producers and consumers;
- public semantic contracts;
- clock-domain rules;
- source / incarnation / reconnect semantics;
- historical PR conclusions and migration classification;
- explicitly rejected approaches;
- tests already executed successfully;
- files known to be out of scope.

Do not rerun successful expensive validation unless:

- relevant code changed afterward;
- a dependency affecting that contract changed;
- the environment or generated artifacts changed in a relevant way;
- a later failure calls the previous result into question;
- final release or integration validation explicitly requires it.

When revalidation is required, rerun the narrowest affected layer first.

#### Do not re-litigate settled decisions

Semantic decisions that have already been investigated, justified, and written
to the checkpoint or durable shared analysis must not be repeatedly reconsidered
without new evidence.

This applies especially to:

- TF ownership;
- clock-domain contracts;
- freshness semantics;
- state ownership and invalidation;
- source-incarnation and reconnect rules;
- sparse configuration semantics;
- firmware-versus-host safety authority;
- receiver-specific policy;
- public API compatibility;
- historical implementation archaeology;
- explicitly rejected legacy architectures.

A context compaction, model change, or new agent session is not new evidence.

#### Investigation freeze

Before substantial implementation begins, summarize the established contract
and authorized implementation plan in the checkpoint.

Once implementation starts, do not return to broad exploratory mode unless:

- implementation exposes a contradiction;
- a deterministic test disproves an established assumption;
- required information is genuinely missing;
- repository state changed enough to invalidate previous evidence.

Interesting neighbouring improvements are not sufficient reason to reopen broad
investigation.

Record them as follow-up findings and continue the authorized task.

#### Validation ladder

Validation should progress from cheap and focused to expensive and broad.

Typical order:

1. static inspection or exact contract check;
2. focused regression test;
3. affected component/package tests;
4. build of directly affected binaries/packages;
5. cross-language or generated-binding equivalence checks when relevant;
6. integration / launch / ROS2 tests;
7. replay, sanitizer, hardware, or simulation validation when relevant;
8. broader monorepo validation only when justified.

Do not run the most expensive validation first unless the task specifically
requires it.

If an early validation step fails, investigate that failure before spending
resources on broader validation.

#### Evidence invalidation rule

Previously established evidence remains valid until something relevant changes.

Do not invalidate evidence merely because:

- time passed during the same task;
- context was compacted;
- another agent session started;
- the agent no longer remembers the original reasoning;
- a different model or reasoning tier is now executing the task.

Evidence must be reconsidered when:

- a file involved in the proven contract changed;
- a dependency affecting that contract changed;
- repository `HEAD`, submodule commit, or base changed in a relevant way;
- build configuration or environment affecting the result changed;
- generated interfaces changed;
- new evidence contradicts the previous conclusion.

When uncertain, determine whether the dependency changed before rerunning an
expensive validation.

#### Search and reading economy

Use repository search as an index, not as an excuse to read everything.

When a symbol, field, topic, service, parameter, command, message, test, or
configuration key is known:

1. search for that exact term;
2. identify producers, consumers, configuration sources, and tests;
3. inspect only relevant contexts;
4. widen the search only if the dependency graph remains unclear.

Avoid reopening the same large file repeatedly.

Record relevant files and symbols in the checkpoint once they are understood.

When historical PRs, forks, or commits are required, first state the precise
behavioural question they are being used to answer.

Do not browse history without a defined question.

#### Test execution economy

Do not run multiple broad suites merely for reassurance.

Use dependency-aware test selection:

- changed leaf implementation -> focused unit test first;
- changed shared library -> focused tests plus affected consumers;
- changed public contract -> all known contract consumers;
- changed `.msg` / `.srv` / generated interface -> generation plus affected
  language bindings and consumers;
- changed ROS2 node/launch/QoS -> affected node and integration tests;
- changed transport/parser/state machine -> focused regression plus relevant
  replay/sanitizer coverage;
- changed firmware behaviour -> firmware tests plus host-interface contract
  checks where applicable;
- final release/integration gate -> broad suite only when explicitly warranted.

If no relevant source changed after a successful expensive test, preserve that
result as valid evidence.

Do not rerun identical expensive tests solely because documentation,
checkpoint files, or unrelated code changed.

#### Avoid speculative implementation

Do not write production code merely to test a theory that can be resolved more
cheaply by:

- inspecting an existing test;
- checking the owning API;
- tracing the exact producer/consumer path;
- reading the relevant project reference;
- examining a small historical diff;
- reproducing the failure with a focused test.

Use implementation as evidence only when cheaper evidence is insufficient.

#### Reasoning-tier discipline

If the execution platform offers multiple models, reasoning tiers, or cost
levels, use the least expensive tier that can reliably perform the current
work.

Prefer lower-cost reasoning for:

- deterministic mechanical edits;
- straightforward regression additions;
- formatting;
- documentation synchronization;
- known API plumbing;
- repetitive implementation with an already established contract;
- routine compilation and validation follow-through.

Reserve higher-cost reasoning for work that materially benefits from it, such
as:

- semantic ambiguity;
- architecture-sensitive changes;
- difficult defect isolation;
- historical implementation archaeology;
- risky compatibility analysis;
- concurrency, timing, ownership, freshness, provenance, or lifecycle
  semantics;
- final surgical review of high-impact changes.

Do not repeatedly escalate and de-escalate reasoning tiers without a concrete
reason.

When an expensive reasoning pass establishes a semantic decision, preserve the
decision and evidence in the checkpoint so cheaper subsequent work can use it
without reproducing the analysis.

#### Protect the final validation budget

Do not consume nearly all available reasoning/context budget during exploration
or implementation and leave insufficient capacity for:

- reviewing the resulting diff;
- running targeted regressions;
- checking unintended scope expansion;
- confirming architecture and safety invariants;
- updating durable analysis / audit ledgers;
- updating the checkpoint;
- producing an accurate handoff.

Reserve enough capacity to prove and report the work.

Implementation without sufficient remaining capacity for validation is not a
successful completion.

#### Low-budget mode

When the platform exposes remaining context, reasoning, execution, or quota,
take increasingly conservative action as remaining budget decreases.

When no explicit indicator exists, enter low-budget mode when signs include:

- repeated context compaction;
- a very long-running agent session;
- substantial repository archaeology already performed;
- many large tool results accumulated;
- validation still pending after significant implementation work.

In low-budget mode:

1. stop broad exploration;
2. stop optional refactoring and cleanup;
3. update the checkpoint immediately;
4. preserve semantic decisions and validation evidence;
5. finish only the smallest safe atomic operation already in progress;
6. prefer targeted tests over broad suites;
7. do not open a new finding, subsystem, or feature;
8. do not begin another repository-wide audit;
9. leave an exact next command/action when work must continue later.

If exact remaining budget is visible and falls below roughly 20%, prioritize
checkpoint consolidation and closure over additional exploration.

#### Critical-budget mode

If exact remaining budget falls below roughly 10%, or available capacity is
otherwise clearly critical, prioritize recoverability over completion.

Immediately record:

- repository, branch, and current `HEAD`;
- relevant submodule/gitlink commits;
- dirty and untracked files;
- current objective;
- completed implementation;
- validation already passed;
- validation still required;
- unresolved questions;
- exact next step.

Then perform only low-risk closure work unless the current atomic task can
clearly finish and be validated within the remaining budget.

Do not use the final available budget attempting speculative fixes,
repository-wide searches, broad test suites, or unrelated cleanup.

If implementation is complete but validation is incomplete, report it as
implemented but not yet validated.

Never convert incomplete evidence into a successful completion claim merely
because the session is ending.

Do not consume the final available budget merely because it exists.

#### Context compaction and restart recovery

After automatic context compaction, model switch, devcontainer rebuild, or a
restarted agent session:

1. read the required project rules according to the top of this file;
2. read the latest durable checkpoint;
3. verify repository path, branch, `HEAD`, dirty state, remotes, and relevant
   submodule/gitlink state;
4. compare actual state with the checkpoint;
5. inspect only information required to resolve discrepancies;
6. continue from the recorded exact next step.

Do not reconstruct the entire task by rereading the monorepo.

Do not repeat historical archaeology already summarized in the checkpoint or
durable shared analysis unless repository state or new evidence contradicts it.

#### Output and tool-call economy

Do not generate long narrative progress reports during active implementation
unless required for a checkpoint, decision record, or handoff.

Prefer concise progress state containing:

- what was established;
- what changed;
- what passed;
- what remains.

Avoid repeatedly printing large diffs, logs, generated files, or successful
test output when a concise summary is sufficient.

For successful commands, preserve the command, exit status, and meaningful
result rather than unnecessary full output.

For failures, retain enough output to diagnose the failure accurately.

#### Preserve a reasoning cache

The combination of:

```text
AGENTS.md
CLAUDE.md and subsystem references
.agent/checkpoints/
.agent/shared/
Git state
tests and generated-interface checks
versioned audits / architecture documents
```

forms the durable reasoning cache for a task.

Use each layer for its intended purpose:

- `AGENTS.md` stores stable agent execution policy;
- `CLAUDE.md` and its references store current project architecture and safety
  contracts;
- local checkpoints store compact current-task conclusions and next state;
- `.agent/shared/` stores intentionally shared short-lived handoff state;
- Git stores exact implementation state;
- tests store executable behavioural evidence;
- versioned audits/architecture documents store durable cross-session findings.

Do not duplicate large amounts of information between these layers.

Thinking deeply once and preserving the conclusion is preferred over repeatedly
reconstructing the same reasoning.

#### Stop conditions

Stop expanding the task when the authorized contract is satisfied.

Do not continue modifying code merely because additional improvements are
visible.

New findings discovered during implementation should normally be:

1. documented;
2. classified;
3. left for a separate authorized task,

unless they are required for correctness or safety of the current change.

A partially completed task must always remain resumable from repository state,
checkpoint state, and recorded validation evidence.

### Resource exhaustion failsafe

If the remaining model or context budget becomes visibly low:

- stop opening new investigative branches;
- do not begin a new production-code subsystem;
- finish the current atomic operation when safe;
- update the checkpoint immediately;
- record exact next commands, files, questions, and unresolved decisions needed
  to resume;
- stop before leaving a knowingly partial architectural change.

If the checkpoint is complete enough for deterministic continuation, stopping
early is preferable to beginning work that cannot be safely completed.

---


## Collaborative analysis and work sharing

Large analyses should be structured so findings can be safely distributed among
contributors without requiring each contributor or agent to rediscover the
entire repository.

This repository may be worked on by multiple human contributors and multiple
agents. Shared state must therefore distinguish temporary collaboration from
durable project knowledge.

When a broad analysis identifies multiple independent findings or subsystems:

- record the shared architectural baseline first;
- record common invariants and contracts once;
- split work only after ownership and dependency boundaries are understood;
- give each finding a stable identifier when practical;
- record severity, evidence, scope, dependencies, and remediation status;
- identify which findings can proceed independently;
- identify ordering constraints between findings;
- keep the shared analysis or ledger updated as remediation progresses.

A contributor or agent receiving a subtask should be able to determine:

- what is already known;
- what has already been validated;
- what must not be changed;
- which repository baseline the conclusions refer to;
- which dependencies are already fixed;
- which findings remain open;
- which tests establish completion;
- what exact next action is expected.

### Audit and backlog classification model

For substantial repository audits, TODO reviews, remediation matrices, and
backlog reconciliation, classify **task status** separately from **task type or
scope**.

Do not mix lifecycle state with subsystem ownership or category.

Recommended status values are:

```text
IMPLEMENTED
PARTIAL
OPEN
BLOCKED
SUPERSEDED
OBSOLETE
DUPLICATE
```

Recommended MowgliNext type/scope values include, as applicable:

```text
ROS2
NAVIGATION
LOCALIZATION
GNSS
GUI
BACKEND
FRONTEND
FIRMWARE
HARDWARE_BRIDGE
CONFIGURATION
DEPLOYMENT
SUBMODULE
DOCUMENTATION
VALIDATION
DOWNSTREAM
```

Additional subsystem-specific scope values may be used when they improve
clarity, but they must not replace the status field.

Examples:

```text
Status: OPEN
Scope: DOCUMENTATION
```

```text
Status: BLOCKED
Scope: GNSS
```

A documentation task is not complete merely because its scope is
`DOCUMENTATION`. A submodule-related task is not necessarily blocked merely
because it belongs to `SUBMODULE`.

### Audit conservation and duplicate invariants

Before editing a backlog after classification, freeze the classification and
verify mechanical conservation.

At minimum:

```text
original item count = sum(all classified items)
```

After cleanup:

```text
original item count = remaining items + intentionally removed items
```

Every intentionally removed item must retain enough traceability in the audit
ledger to identify:

- its original stable identity or text;
- its status and scope;
- the evidence supporting removal;
- its final disposition.

`DUPLICATE` is directional. Every duplicate item must reference exactly one
canonical item that is itself not classified `DUPLICATE`.

Invalid:

```text
A -> DUPLICATE of B
B -> DUPLICATE of A
```

Valid:

```text
A -> OPEN / PARTIAL / IMPLEMENTED / ...
B -> DUPLICATE of A
```

Before applying backlog edits, validate both:

1. the numeric count invariant;
2. the canonical-reference invariant for duplicates and dependencies.

After cleanup, recount the resulting backlog and confirm that the expected
number of retained and removed items matches the frozen ledger.

### Strict PARTIAL and BLOCKED semantics

Use `PARTIAL` only when part of the **same requested contract** already exists.
Neighbouring infrastructure, reusable groundwork, a related ROS2/GUI surface, or
a similar implementation in another subsystem does not by itself make an item
`PARTIAL`.

Every `PARTIAL` classification should record:

```text
Existing:
Missing:
Completion criterion:
```

Use `BLOCKED` only when a real dependency prevents safe or meaningful progress.
Do not use `BLOCKED` merely to express preferred ordering, lower priority, or a
future phase.

Every `BLOCKED` classification should record:

```text
Blocked by: <stable item/finding ID or explicit external dependency>
Unblocks when: <observable completion condition>
```

If work can proceed independently but should simply happen later, classify it
`OPEN` and record the ordering recommendation separately.

### Stable audit and backlog identifiers

For substantial audits or large backlogs, give meaningful findings/items stable
identifiers before they are used as dependency targets or shared across
contributors.

Examples:

```text
MGNSS-008
MN-ROS2-012
MN-GUI-004
MN-FW-003
MN-NAV-006
```

Preserve established project-specific identifiers when they already exist. Do
not renumber mature findings merely to fit a new naming scheme.

Line numbers and current Markdown positions may be recorded as secondary
baseline information, but must not be the primary identity because backlog edits
move them.

When an item is renamed, moved, deduplicated, or transferred between Universal
GNSS and MowgliNext ownership, preserve its stable identity or record an explicit
canonical replacement/relationship.

A dependency graph, remediation matrix, or shared handoff should reference
stable IDs rather than mutable line numbers whenever practical.

### Audit freeze and contradiction review

For large backlog audits, prefer this sequence:

```text
baseline
    -> targeted evidence collection
    -> item classification
    -> classification freeze
    -> canonical duplicate/dependency validation
    -> mechanical count validation
    -> contradiction review
    -> exact cleanup plan
    -> apply authorized backlog/documentation edits
    -> post-cleanup conservation check
```

Do not edit the authoritative backlog while classifications are still changing
unless the task explicitly requires an incremental live ledger.

When the audit discovers stale durable documentation, record the contradiction
and classify the affected evidence source as `PARTIALLY_STALE` or `SUPERSEDED`
as appropriate. Either repair it within authorized scope or leave an explicit
follow-up finding so future agents do not reuse known-stale text as current
truth.

When a contradiction crosses the parent repository and a submodule, record the
parent `HEAD`, parent gitlink, submodule worktree `HEAD`, and authoritative
upstream evidence before deciding which side is stale.

### Sharing an unfinished task

If another contributor must resume work before the analysis is mature enough
for permanent documentation, create or update a concise handoff under:

```text
.agent/shared/
```

The handoff should reference any relevant local or durable evidence without
copying large logs or full analyses.

When the receiving contributor resumes:

1. read the shared handoff before broad exploration;
2. revalidate repository/branch/HEAD/submodule state;
3. reuse still-valid evidence;
4. investigate only missing, ambiguous, stale, or invalidated conclusions;
5. update or retire the handoff as ownership changes.

### Sharing a large audit

For a large audit shared between contributors:

- keep the authoritative finding ledger in versioned documentation when it is
  already mature enough to be durable;
- use `.agent/shared/` only for temporary cross-contributor execution state;
- keep local session checkpoints in `.agent/checkpoints/`;
- avoid maintaining separate incompatible copies of the same findings;
- promote validated conclusions to `docs/audits/`, `docs/analysis/`, or
  `docs/architecture/` as soon as they are stable and reusable.

Do not duplicate expensive repository-wide analysis across parallel
contributors when a validated shared baseline already exists.

Do not commit every local checkpoint merely to make it shareable. Share only
the minimum state required for collaboration.

---

## Validation discipline

For every implementation:

- Run focused tests first.
- Expand validation only as needed.
- Distinguish pre-existing baseline failures from regressions introduced by the task.
- Verify formatting for all touched languages and components.
- Run `git diff --check`.
- Finish with an explicit repository status snapshot.

Never claim a finding fixed without deterministic validation covering its acceptance criteria.

### Regression-first remediation

When fixing a confirmed defect, define the failure and acceptance matrix before changing production behavior.

Whenever practical, add or identify a deterministic regression that reproduces the violated invariant before implementing the fix.

Do not weaken an existing test merely because a new implementation makes it fail.

When a test fails after a change, first determine whether:

- the implementation violates the established contract, or
- the test expectation itself is incorrect or obsolete.

Document that conclusion before changing the test or production behavior.

---

## Git safety

Unless explicitly instructed otherwise:

- Do not commit automatically.
- Do not push automatically.
- Do not modify unrelated files.
- Preserve unrelated untracked files.
- Do not mix generated environment noise into the patch.
- Do not silently update submodules.
- Do not silently change gitlinks.
- Verify branch, HEAD, remotes, and relevant submodule state before large changes.
- Revalidate repository state before concluding long-running work.

- Never stage `.agent/checkpoints/` or other local agent state.
- Stage `.agent/shared/` only when the shared handoff is intentionally meant to
  be exchanged through Git.
- Review staged `.agent/shared/` content for secrets, stale session noise, large
  logs, and accidental local-only state before committing it.
- Prefer promoting mature reusable knowledge to versioned documentation rather
  than keeping permanent handoffs under `.agent/shared/`.

If repository state differs from an explicitly required baseline, stop and report the mismatch before making production changes.

---

## Scope and completion discipline

### Atomic task boundaries

Prefer completing one coherent finding, subsystem, or remediation phase before starting another.

Do not opportunistically fix unrelated defects discovered during a scoped task.

Record unrelated findings separately unless they block the requested work.

Do not expand scope merely because adjacent code appears improvable.

### Definition of done

A task is not complete merely because the implementation compiles.

Before declaring completion, verify as applicable:

- requested behavior is implemented
- acceptance criteria are satisfied
- adversarial and failure-path regressions pass
- existing affected tests remain green
- generated bindings are reproducible
- C++ / Python / Go / frontend implementations remain equivalent where required
- formatting is clean
- `git diff --check` is clean
- unrelated files are untouched
- relevant submodules remain at the expected commits
- repository status is explicitly reported
- residual limitations are documented
- remaining open findings or follow-up work are identified accurately

For substantial audits or architecture studies, also verify whether validated reusable knowledge should be promoted into versioned shared documentation before closing the task.

For backlog reconciliation audits, completion also requires the applicable
classification-count, duplicate-canonicalization, dependency-reference, and
post-cleanup conservation checks from the collaborative-analysis rules above to
pass or to be explicitly reported as unresolved.

### No silent scope expansion

If a correct fix requires changing a component declared read-only or out-of-scope, stop and explain the dependency before modifying it.

Do not silently weaken an invariant, compatibility requirement, safety boundary, or test expectation in order to complete the requested scope.

---

## Handoff discipline

Every final response for implementation, audit, migration, debugging, or
architecture work must explicitly report, as applicable:

- files modified;
- why each file changed;
- tests executed;
- validation performed;
- important evidence established;
- remaining known limitations;
- relevant open findings or follow-up work;
- current repository, branch, and `HEAD`;
- relevant submodule worktree commit and parent gitlink when applicable;
- commit status;
- push status.

Never omit changes performed during the task.

If the task is intentionally left incomplete, also report:

- what remains;
- why it remains;
- the latest local checkpoint;
- whether a `.agent/shared/` handoff was created or updated for another
  contributor;
- validation still pending;
- the exact recommended next step or command.

Do not describe a task as complete while required validation is still running,
blocked, or unknown.
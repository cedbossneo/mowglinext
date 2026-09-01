# MIGRATION.md — historical and semantic migrations

Load this cumulative module when consulting a historical PR, fork, branch,
commit, or patch; performing implementation archaeology or compatibility
comparison; or porting a semantic change. Load other modules for the affected
subsystem.

## Defined behavioural question first

Before browsing history, state the precise behavioural question it must answer.
Do not inspect history without one. Historical changes are behavioural evidence,
not automatically mergeable code and not current authority.

Record current repository/branch/`HEAD`, base, relevant gitlink/submodule state,
historical source/commit, target contract, and compatibility/safety constraints.
Compare historical behaviour with current architecture, source, tests,
`CLAUDE.md` invariants, and subsystem references. Never claim an old change
remains applicable without this comparison.

## Mandatory disposition

Classify every historical change considered:

- `PORT`: semantics and architecture remain applicable; carry it with only
  mechanical adjustment.
- `ADAPT`: behavioural intent remains applicable but current ownership,
  lifecycle, interface, safety, or architecture requires redesign.
- `ALREADY PRESENT`: current implementation already satisfies the contract;
  cite exact evidence and do not duplicate it.
- `SUPERSEDED`: a newer architecture/contract replaces it; identify replacement
  evidence.
- `REJECT`: incorrect, unsafe, incompatible, out of scope, or no longer desired;
  record a concise reason.

Preserve question, classification, evidence, and invalidation condition in the
migration ledger/checkpoint. Do not blend classifications or port code before
ownership/lifecycle/provenance and compatibility semantics are established.

## Migration validation

Define semantic equivalence and intentional differences before edits. Validate
the smallest affected layer first, then public consumers, generated interfaces,
cross-language equivalents, integration, and hardware/field evidence only as
applicable. Historical tests are evidence but must be checked against the
current contract; never weaken current tests merely to imitate legacy code.

Report all classifications, unresolved compatibility risks, current baseline,
validation, and residual limitations in the final handoff.

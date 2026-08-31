# MowgliNext GNSS Downstream Audit

> Final audit report and remediation ledger. Phase A, Phase B, Phase C, Phase D,
> Phase E, and Phase F implementation status is recorded separately from the original
> findings so open consumers are not accidentally presented as fixed.

## Executive summary

Audit status: **complete**.

The hard pre-flight passed: the top-level checkout was clean on
`audit/gnss-downstream`, and the Universal GNSS submodule was clean and pinned
to `45fe44a031520f9dfe4bfc07fd515952d1a1ea88` before Phase A edits began.
Phase B started from clean HEAD `93475cdc`, containing the committed Phase A
changes, with the same exact clean Universal GNSS baseline.
Phase C started from clean HEAD `1c586082`, containing the committed Phase A
and Phase B changes, again with the exact clean pinned submodule.
Phase D started from clean HEAD `97c75ebe`, containing committed Phases A-C,
again on the exact clean pinned Universal GNSS baseline.
Phase E validation resumed from parent HEAD
`a14cf585cd8626ceb54aec592e07eaafed2b7e07`, with transferred Universal GNSS
changes still based on the exact pinned gitlink
`45fe44a031520f9dfe4bfc07fd515952d1a1ea88` and no submodule or parent commit.
Phase F started from clean parent HEAD
`c6cd8b907bb43bb42611976e51e39cb2deca45c9` with Universal GNSS read-only and
clean at the exact gitlink `5281472116669972ae12b9d1997d66b064671cf5`.

The audit found that Universal GNSS distinguishes a new receiver observation
from timer-driven republication using `position_observation_sequence`, while
MowgliNext discarded that identity and several consumers treated callback
arrival as physical acquisition.

Phase A now preserves the upstream sequence and receipt stamp through both
public bridges. FusionGraph continues to consume only `/gps/fix`, but uses its
pinned-contract receiver-receipt stamp to reject cached, delayed-duplicate,
zero/future, and old-clock-epoch publications before factor insertion or RTK
freshness/streak changes. Behavior, dig trust, the hardware lock indication,
and monitoring still use their old callback/delivery freshness logic and remain
open.

Phase B replaces NavSat projection's authoritative latest-status cache with an
exact, bounded receipt-provenance rendezvous. Both callback orders are accepted,
typed RTK classification is released only for a fresh unambiguous fix/status
pair with the same receipt, and sequence-aware duplicate/ambiguity/restart rules
prevent cross-epoch reuse. A fix whose status never arrives may still produce
the semantically safe `/gps/absolute_pose` path from its own `NavSatStatus`
after the bounded wait, but it can never authorize `/gps/pose_cov`.

Phase C wires the existing physical receiver-profile setting
`gnss_profile_rate_hz` into `cog_to_imu` and derives its default maximum
receipt interval as `1.5 / physical_rate_hz`. Genuine 1/5/7/10 Hz observations
therefore retain a bounded displacement baseline, while cached receipt stamps
remain inert and gaps of two or more expected periods reseed. Receipt/clock
rewinds fail closed, and the stationary latch now requires valid ROS provenance
and bounded monotonic age instead of clamping negative ROS age fresh.

Phase D applies the shared observation identity to the four remaining local
consumers. Behavior reevaluates its existing 5 s deadline before each tree tick;
dig trust and the firmware-bound lock quality share the existing hardware 2 s
deadline; localization monitoring keeps its existing 2 s deadline and separates
cached callback liveness from physical observation freshness. In all four,
cached delivery cannot refresh authority, invalid/future provenance fails
closed, and ROS/monotonic epoch rewinds isolate old semantic state.

Phase E restores the field-proven Unicore rover policy at receiver configuration
boundaries. UM980 now defaults to `MODE ROVER UAV`; UM960, UM982, and UB9A0
retain `MODE ROVER SURVEY MOW`; the guarded generic fallback remains
`MODE ROVER`. Optional expert overrides for rover dynamic mode and the bounded
RTK/DGPS correction-age windows flow through Universal GNSS PLAN/APPLY, the Go
API, settings persistence, web controls, and startup. The defaults are RTK
120 s and DGPS 300 s, while `RTK RELIABILITY 3 1` remains unchanged.

Phase F replaces indefinite correction diagnostic entries with complete,
source-owned NtripNode and ReceiverNode snapshots. A 2 s steady-clock receipt
deadline bounds authority; diagnostic ROS stamps order snapshots but never
provide liveness. Source changes, disconnect/reconnect, station mismatch,
snapshot omission, explicit CLEAR, and expiry fail closed. The public contract
now separates NTRIP transport/response, valid RTCM flow, forwarding, MSM
observation, and dynamic semantic health. Browser merge preserves typed CLEAR,
and onboarding requires current valid dynamic semantic evidence rather than a
generic `ACTIVE` forwarding string.

Original audit result: **11 confirmed findings — 0 P0, 6 P1, 5 P2, 0 P3**. The P1 set is
the FusionGraph duplicate-observation path, behavior safety-health liveness,
dig-event trust, UM980 moving-rover configuration regression, ineffective
Unicore correction-age policy, and the supported 1 Hz COG dead zone. The P2 set
covers asynchronous NavSat/status pairing, the latched lock indication,
monitoring provenance, backend/browser stale state, and correction-state
projection.

The dominant architectural defect is the loss of observation identity and
bounded provenance at the public boundary, followed by downstream consumers
treating delivery, publication, or callback time as physical acquisition. The
independent configuration regressions show that upgrading the pinned submodule
also removed Mowgli-specific receiver behavior without removing its GUI/startup
contract.

Phases A through F change production code only for the public contract, both
bridges, FusionGraph deduplication/freshness, the shared freshness primitive,
the NavSat projection association targeted by MGNSS-002, and COG timing targeted
by MGNSS-011, the four local consumers targeted by MGNSS-003/004/006/007, and
the receiver-configuration boundaries targeted by MGNSS-005/010, and the
correction projection/readiness boundary targeted by MGNSS-009.

Current remediation result: **10 fixed, 1 open**. The remaining finding is
exactly MGNSS-008. Historical audit totals and severities remain unchanged.

## Phase A remediation status

Implemented 2026-08-30 as the first narrow fix batch:

| Finding | Phase A state | Result |
|---|---|---|
| MGNSS-001 | **FIXED** | Public sequence preserved; FusionGraph receipt-provenance deduplication occurs before all factor/RTK evidence effects; all RTK-current gates reject negative age. |
| MGNSS-003 | **ENABLED FOR NEXT PHASE — OPEN** | Behavior can now consume public sequence and shared freshness policy, but still refreshes health/debounce from callback `now()`. |
| MGNSS-004 | **ENABLED FOR NEXT PHASE — OPEN** | Hardware dig trust can now consume the shared contract, but its callback-time freshness was not changed. |
| MGNSS-006 | **ENABLED FOR NEXT PHASE — OPEN** | Lock indication can now identify new/stale observations, but quality expiry was not implemented. |
| MGNSS-007 | **ENABLED FOR NEXT PHASE — OPEN** | Monitoring has the reusable receipt/monotonic policy available, but still monitors `/gps/fix` callback delivery. |

In Phase A, MGNSS-002, 005, 008, 009, 010, and 011 were intentionally
untouched.

## Phase B remediation status

Implemented 2026-08-30 as the second narrow fix batch:

| Finding | Phase B state | Result |
|---|---|---|
| MGNSS-002 | **FIXED** | The unbounded latest-status cache is removed. Exact receipt-stamp pairing, non-zero sequence identity, monotonic bounded waiting/capacity, fail-closed clock/provenance/reset behavior, and both callback orders are covered by deterministic regressions. |

MGNSS-003 through MGNSS-011 were not changed by Phase B. The original severity
and finding count remain historical audit facts.

## Phase C remediation status

Implemented 2026-08-30 as the third narrow fix batch:

| Finding | Phase C state | Result |
|---|---|---|
| MGNSS-011 | **FIXED** | COG timing is derived from configured physical acquisition cadence, all exposed 1/5/7/10 Hz rates retain genuine samples, cached/delayed duplicates remain inert, gaps/restarts/clock errors reseed fail-closed, and the stationary latch rejects negative age and expires monotonically. |

MGNSS-003 through MGNSS-010 were not changed by Phase C. The original severity
and finding count remain historical audit facts.

## Phase D remediation status

Implemented and validated 2026-08-30 as one shared-consumer remediation batch:

| Finding | Phase D state | Result |
|---|---|---|
| MGNSS-003 | **FIXED** | Behavior consumes public receipt+sequence identity, reevaluates the existing 5 s observation deadline before every tree tick, immediately clears Fixed-only authority/stales the health gate at expiry, and recovers only from accepted evidence. |
| MGNSS-004 | **FIXED** | Hardware dig trust uses shared receipt/sequence plus monotonic physical-observation freshness; cached delivery cannot refresh the existing 2 s gate or establish a new dig anchor. |
| MGNSS-006 | **FIXED** | The same hardware verdict expires firmware-bound quality to established value `0` (LOCK LED off) and restores the existing quality mapping only on genuine observations. |
| MGNSS-007 | **FIXED** | The launched localization monitor evaluates `/gps/absolute_pose` receiver-receipt provenance rather than callback arrival and reports callback liveness separately on freshness transitions. |

The generic Phase A helper now has one consumer-facing wrapper: ROS receipt age
and monotonic elapsed age since the last accepted identity must both pass.
Cached publications update delivery liveness only. Invalid/future provenance,
ROS/monotonic rewind, and receipt-only source rewind fail closed. Thirty-two new
deterministic cases cover the required consumer and cross-cutting matrix; all
three affected package builds and their complete GTest suites pass.

## Phase E remediation status

Implemented and validated 2026-08-30 as one receiver-configuration remediation
batch:

| Finding | Phase E state | Result |
|---|---|---|
| MGNSS-005 | **FIXED** | Universal GNSS restores the model-aware UM980 UAV default, preserves the established Survey Mow and guarded generic defaults, and accepts explicit rover dynamic-mode overrides through PLAN/APPLY, the Go API, and startup. |
| MGNSS-010 | **FIXED** | Universal GNSS restores RTK 120 s / DGPS 300 s while preserving reliability 3 1; optional 1..1800 s expert overrides are validated and wired through schema, persistence, web controls, Go PLAN/APPLY, and startup. |

Focused isolated tests cover the model defaults, explicit overrides, timeout
bounds, PLAN/APPLY equivalence, saved-settings propagation, omitted-setting
behavior, and startup ordering. No historical frame writer, RTCM forwarding,
NTRIP/correction-health logic, or GLONASS-1230 requirement was ported. MGNSS-008
and MGNSS-009 remain out of scope and open.

## Phase F remediation status

Implemented and validated 2026-08-31 as the correction-projection remediation
batch:

| Finding | Phase F state | Result |
|---|---|---|
| MGNSS-009 | **FIXED** | Complete per-owner snapshots, monotonic 2 s expiry, source/station/lifecycle invalidation, receiver-first forwarding ownership, distinct typed transport/response/flow/semantic fields, correction-specific SET/OMIT/CLEAR merge rules, and semantic onboarding readiness are covered by deterministic C++/Python/frontend regressions. |

ReceiverNode owns confirmed receiver writes; NtripNode owns transport, accepted
response, valid RTCM arrival, and its portable-RTK semantic truth. A current
ReceiverNode snapshot can supply receiver forwarding and MSM display when
NTRIP is absent/non-streaming, but it cannot fabricate NTRIP transport or
semantic health. GLONASS 1230 remains optional. Generic backend/Foxglove/
browser connection-lifetime invalidation was not changed and remains exactly
MGNSS-008.

## Baselines

### Required pre-flight

| Check | Revalidated result |
|---|---|
| Branch | `audit/gnss-downstream` |
| Top-level worktree | clean before report creation |
| Top-level HEAD | `bc8b1de3 test: pin audited Universal GNSS for downstream audit` |
| Universal GNSS gitlink/HEAD | `45fe44a031520f9dfe4bfc07fd515952d1a1ea88` |
| Universal GNSS worktree | clean |
| Legacy comparison commit | `4bf5d0a291816285ef96f7fba56b623136ab8f41`, available locally |

### Original audit constraints

- Mode: audit only.
- Allowed modified file: `MOWGLINEXT_GNSS_AUDIT.md` only.
- No production fixes, refactors, commits, or pushes.
- Builds/tests must run as the normal project user, never as root.

### Phase A implementation constraints

- Mode: narrow fix + test.
- Universal GNSS remains read-only at the pinned commit.
- No UM980, correction-age, COG, NTRIP/RTCM projection, backend/UI, or unrelated
  consumer fix is included.
- No commit or push.

### Phase B implementation constraints

- Mode: narrow fix + test for MGNSS-002 only.
- Universal GNSS remains read-only at the pinned commit.
- Existing topics, coordinate conversion, frames, covariance thresholds, and
  unrelated consumers remain unchanged.
- No commit or push.

### Phase C implementation constraints

- Mode: narrow fix + test for MGNSS-011 only.
- Universal GNSS remains read-only at the pinned commit.
- COG topic/frame/heading/covariance/displacement/speed/rotation semantics remain
  unchanged; only timing and temporal-state validity change.
- No commit or push.

## GNSS data-flow map

Current production path, revalidated from source:

```text
physical receiver
  -> Universal GNSS ReceiverNode
       -> /_gps_internal/universal/status  (receipt stamp + observation sequence)
       -> /gps/fix                         (receipt stamp, while runtime state is fresh)
       -> /diagnostics
  -> mowgli_gnss_bridge
       -> /gps/status                      (receipt stamp + sequence preserved)
       -> /rtcm
  -> FusionGraph / GTSAM                  (/gps/fix directly)
  -> cog_to_imu                           (physical-rate receipt timing
                                           /gps/fix -> /imu/cog_heading)
  -> navsat_to_absolute_pose              (exact bounded receipt-paired fix/status)
       -> /gps/absolute_pose
       -> /gps/pose_cov
  -> behavior/localization-health         (/gps/status)
  -> hardware bridge                      (/gps/status; LED and dig trust)
  -> localization monitoring              (/gps/absolute_pose receipt provenance)
  -> map/dock calibration                 (/gps/pose_cov)
  -> Foxglove bridge -> Go RosProvider -> GUI WebSocket -> React GNSS state
```

Correction path:

```text
caster/NTRIP
  -> Universal GNSS NtripNode
       -> validated RTCM frames
       -> forwarding, transport, and semantic diagnostics
  -> ReceiverNode RTCM input / receiver solution state
  -> bridge diagnostic cache/projection
       -> public correction_stream_status + MSM summary in /gps/status
  -> setup/readiness UI and GNSS diagnostics UI
```

The upstream `ReceiverNode::PublishNow()` publishes `/gps/fix` before
`/gps/status` and publishes status on every publication timer. NavSatFix is
suppressed after the monotonic runtime-observation freshness window expires;
typed status continues to be published from the current cached runtime state.

## Universal GNSS contract matrix

| Contract | Upstream behavior | Current downstream result | Status |
|---|---|---|---|
| Receipt provenance stamp | `GnssStatus.stamp` and `NavSatFix.header.stamp` are local receiver acceptance/receipt time in ROS-clock domain, not publication or callback time. | Bridge preserves public `header.stamp`; FusionGraph, NavSat projection, COG, behavior, hardware dig/LED, and localization monitoring now use preserved receipt provenance. | fixed in all audited local ROS consumers; backend/GUI lifecycle remains separate MGNSS-008 |
| Position observation identity | Sequence advances only for an accepted position observation, including numerically identical observations; cached publication keeps the sequence. | Public status appends the sequence and both bridges copy it verbatim. FusionGraph deduplicates by receipt; NavSat projection handles exact receipt+sequence association; behavior and hardware consume sequence+receipt. Monitoring's `AbsolutePose` interface has no sequence and uses the helper's fail-closed receipt-only mode. | fixed for all interfaces that expose sequence; safe compatibility mode for monitoring |
| Acquisition vs publication | Physical acquisition is independent of `publish_rate_hz`; fresh cached state may be republished. | FusionGraph, projection, COG, behavior, dig trust, LED, and monitoring ignore cached delivery for physical freshness. Explicit 1 Hz acquisition/10 Hz callback and sparse-publication tests pass. | fixed for audited local ROS consumers |
| SET / OMIT / CLEAR | Capability and value masks preserve supported/current-value tri-state semantics. | The bridge reconstructs each core message and copies both masks. Phase F treats an unrelated diagnostic array as OMIT and a represented owner's complete snapshot as SET/CLEAR; typed correction group value masks override browser fallback rather than being OR-resurrected. | fixed in Phase F; generic stream-loss lifetime remains MGNSS-008 |
| Clock domains | Upstream liveness uses steady time; public provenance uses ROS time. | Shared policy separates ROS receipt age, monotonic physical-observation elapsed age, and callback delivery liveness. FusionGraph, projection, COG, behavior, hardware, and monitoring reject future/negative age; Phase D consumers isolate debounce state on epoch rewind. | fixed for all audited local ROS consumers |
| Correction states | Transport, accepted response, valid flow, semantic health, and receiver RTK solution are distinct. | Append-only typed fields and capability/value groups expose transport/response, valid flow, forwarding, MSM, and semantic health separately; readiness requires all current semantic evidence and does not use RTK Fixed as a proxy. | fixed in Phase F |
| RTCM semantics | 1005/1006 are static-base data, MSM is dynamic correction data, and 1230 is optional for normal RTK health. | Phase F consumes upstream portable-RTK truth and checks same-snapshot base/MSM consistency; static-only, malformed, stale, station-mismatched, and zero-cell MSM fail readiness. No 1230 requirement is introduced. | fixed in Phase F; 1230 preserved optional |
| Source/station ownership | Reconnect or source/station change must invalidate incompatible stale state. | Full per-owner snapshots carry upstream hardware identity, expire on steady time, replace omitted values with CLEAR, reject older ROS-stamped resurrection, and never merge NTRIP/receiver semantic epochs. | fixed in Phase F |

## Old-fork compatibility delta

Final classification:

| Legacy behavior/change | Audited baseline | MowgliNext dependency | Classification |
|---|---|---|---|
| `--rover-dynamic-mode`; UM980 default `MODE ROVER UAV` | Phase E restores the model-aware UAV default and portable override while preserving established Survey Mow/generic behavior. | Startup and Go plan/apply pass only explicit overrides; Auto uses the restored model-aware default. | **RESTORED IN PHASE E** |
| Unicore correction-age windows (legacy RTK 120 s, DGPS 300 s) | Phase E restores RTK 120 s and DGPS 300 s while preserving `RTK RELIABILITY 3 1`. | Optional bounded expert settings now flow through schema/UI, persistence, Go plan/apply, startup, and Universal GNSS. | **RESTORED IN PHASE E** |
| Whole-frame RTCM forwarding | Current baseline has a bounded pending-frame FIFO and retries partial/EAGAIN writes without truncating the accepted frame. | No downstream dependency found. | **EQUIVALENT IN NEW BASELINE** |

## Findings summary

Original audit count: **11 findings: 0 P0, 6 P1, 5 P2, 0 P3**.
After Phase F: **10 fixed, 1 open**. The remaining finding is exactly
MGNSS-008.

P1 denotes a high-impact loss of a primary localization/safety function or a
configuration regression with field-proven RTK consequences. P2 denotes a
bounded or secondary-path localization error, misleading health/operator state,
or readiness error without the same immediate primary-path impact.

| ID | Title | Severity | Classification | Evidence state |
|---|---|---:|---|---|
| MGNSS-001 | Observation identity is lost, so FusionGraph reuses cached fixes as fresh factors/evidence | P1 | **fixed in Phase A** | public bridge + receipt-only FusionGraph regressions |
| MGNSS-002 | NavSat projection pairs each fix with an unbounded, asynchronously cached typed status | P2 | **fixed in Phase B** | exact bounded association + 18 focused regressions + NavSat-only launch regression |
| MGNSS-003 | Behavior localization-health freshness follows cached status callback arrival | P1 | **fixed in Phase D** | shared receipt+sequence/steady deadline + behavior safety-boundary regressions |
| MGNSS-004 | Dig-event GNSS trust follows cached status callback arrival | P1 | **fixed in Phase D** | shared receipt+sequence/steady deadline + stale-anchor regressions |
| MGNSS-005 | UM980 dynamic-mode compatibility regressed across the pinned submodule upgrade | P1 | **fixed in Phase E** | model-aware defaults + PLAN/APPLY/API/startup regressions |
| MGNSS-006 | GPS-lock indication does not age receiver observation provenance | P2 | **fixed in Phase D** | shared hardware verdict expires quality to established LED-off value |
| MGNSS-007 | Monitoring reports cached fix delivery as fresh observation health | P2 | **fixed in Phase D** | receipt-only physical freshness + separate delivery-liveness regressions |
| MGNSS-008 | Backend/browser retain old typed GNSS state across stream failures | P2 | confirmed bug | backend cache and frontend lifecycle proof |
| MGNSS-009 | Correction diagnostics cache outlives source/incarnation and collapses flow with health | P2 | **fixed in Phase F** | bounded source snapshots + typed state separation + bridge/browser/readiness regressions |
| MGNSS-010 | Field-proven Unicore correction-age policy was lost and exposed settings are not applied | P1 | **fixed in Phase E** | restored defaults + bounded end-to-end override regressions |
| MGNSS-011 | Supported 1 Hz fixes can never produce COG heading | P1 | **fixed in Phase C** | physical-rate-derived timing + 19 focused regressions + 15 historical COG tests |

This is not a forced reproduction of the recovered 10-finding count: MGNSS-011
was found independently by the full downstream trace.

## Detailed findings

### MGNSS-001 — Observation identity is lost, so FusionGraph reuses cached fixes as fresh factors/evidence

Status: **FIXED in Phase A**.

- Upstream contract: `gnss_ros2/msg/GnssStatus.msg` documents receipt provenance
  at lines 55-58 and observation sequence at lines 59-62.
- Phase A boundary fix: `ros2/src/mowgli_interfaces/msg/GnssStatus.msg` now
  exposes `position_observation_sequence` without reordering existing fields.
  The C++ and Python production bridges copy both the upstream receipt stamp and
  sequence verbatim. Cached publications therefore retain one identity, while
  identical-position genuine observations with a new upstream sequence remain
  distinguishable on the public status contract.
- Phase A consumer fix: FusionGraph still subscribes only to `/gps/fix`, so its
  available stable identity is the preserved receipt stamp. A shared
  `ObservationTracker` gate now runs at the start of `OnGnss()`, before datum,
  wrong-fix, RTK freshness/streak, map-mode, and factor side effects. A cached or
  delayed duplicate is rejected; the same coordinates with a later receipt are
  accepted. Fixed freshness is committed from the accepted receipt stamp, not
  callback `now()`, and only after the observation reaches an intentional
  accepted path (including the documented dock/charge paths or a successful
  queue insertion).
- Original injection proof: `GraphManager::QueueGnss()` overwrites the one pending slot,
  and each graph tick adds that pending `GnssLeverArmFactor` and clears it
  (`graph_manager.cpp:161-170`, `graph_manager_node.cpp:398-420,548`). Thus a
  cached fix published on multiple graph intervals becomes multiple independent
  factors on the same or successive state nodes even though no new physical
  observation occurred.
- Upstream eventually suppresses NavSatFix after its steady-clock runtime
  freshness window, so factor duplication is bounded when acquisition stops;
  within that window, a low-acquisition/high-publication profile can still add
  many copies of one physical observation.
- Before Phase A, the callback-derived RTK time/streak controlled scan-match
  yield, keyframe-map engagement/capture, loop-closure suppression, and COG yaw
  acceptance/recovery. A delayed fixed callback updates those latches before a
  future/evicted measurement timestamp is rejected.
- Phase A clock policy: receipt freshness rejects zero, future, and negative-age
  provenance. A ROS-time rewind resets the Fusion GNSS observation epoch and RTK
  latches fail closed. Monotonic delivery liveness is tracked separately from
  observation freshness so cached callbacks cannot extend physical freshness.
- Violated invariant: publication/callback count must not substitute for new
  position-observation identity.
- Concrete consequence: the graph overweights one physical fix, and localization
  mode gates remain in RTK-Fixed policy according to ROS publication/backlog
  cadence rather than receiver acquisition.
- Phase A regression coverage verifies exact C++/Python bridge agreement,
  repeated cached sequence preservation, distinct identical-position
  observations, one Fusion acceptance/streak update for a repeated receipt, the
  next receipt, delayed duplicates, receiver-sequence restart, receiver silence,
  future stamps, negative age, and ROS-time rewind. The Fusion test exercises
  the same shared tracker used by the callback before its side effects.
- Residual boundary: atomic `/gps/fix` to `/gps/status` association remains
  MGNSS-002 and was intentionally not claimed as fixed in this phase.

### MGNSS-002 — NavSat projection pairs each fix with an unbounded, asynchronously cached typed status

Status: **FIXED in Phase B**.

- Original defect: independent depth-10 subscriptions fed an authoritative,
  unbounded-in-time `last_status_` cache. Every fix used whichever typed status
  callback happened to run last, so Fixed/Float/no-fix transitions could inherit
  the preceding or a backlogged epoch.
- Phase B removes `last_status_` and introduces
  `NavSatStatusAssociation`. The sole cross-topic key is the exact receiver
  receipt stamp; coordinates are never identity. Either callback order creates
  one pending entry, while duplicate fixes and cached same-stamp/same-sequence
  status deliveries cannot create another output.
- Complete pairs are intentionally held until the 250 ms monotonic pairing
  window closes. This makes a same-stamp/different-sequence conflict observable
  before publication; the affected entry is marked ambiguous, withheld, logged,
  and closed. A non-zero public sequence is required for typed authority.
- Pending state is bounded to 32 receipt entries by default. Deadlines use
  `steady_clock`; deterministic capacity overflow closes the oldest delivery.
  Closed, expired, rejected, and evicted receipts cannot be resurrected.
- Receipt validity reuses Phase A's
  `gnss_observation_freshness::IsReceiptFresh`: zero, future, negative-age, and
  older-than-2 s provenance is rejected both on delivery and before release.
  ROS-time or monotonic rewind clears the association epoch and drops the
  triggering delivery. Continuing sequence with receipt rewind is isolated;
  lower sequence at a later receipt starts a receiver-source epoch and retains
  only a fix already waiting on that exact new receipt.
- When the exact status never arrives, a fresh fix expires after the bounded
  wait and publishes `/gps/absolute_pose` once using only its own standard
  NavSat status. It never publishes the typed RTK/covariance
  `/gps/pose_cov` path. Status-only input expires without output.
- Existing coordinates, frames, flags resolution for valid exact pairs,
  covariance gates/thresholds, and output topic names are unchanged. The
  operational trade-off is a bounded 250 ms projection delay, configurable
  together with capacity and maximum provenance age.
- Regression coverage exercises fix-first/status-first, Fixed/Float reversals,
  cached status, duplicate fix, identical coordinates with new identity,
  unrelated and delayed statuses, missing counterparts, expiry/no resurrection,
  deterministic capacity overflow, receipt/ROS/steady epoch rules, invalid and
  future provenance, forward staleness, same-stamp ambiguity, complete
  Fixed/Float/no-fix/Fixed transitions, and receiver sequence restart. A launch
  test verifies the safe fix-only path emits exactly one absolute pose and no
  covariance output.

### MGNSS-003 — Behavior localization-health freshness follows cached status callback arrival

Status: **FIXED IN PHASE D**.

- `BehaviorTreeNode::on_gnss_status()` records `get_clock()->now()` on every
  public status callback and passes that callback time to `LocalizationHealth`,
  rather than using the preserved receiver-receipt stamp or an observation
  sequence (`behavior_tree_node.cpp:427-474`).
- `LocalizationHealth::Update()` treats GNSS as stale only when
  `now - gnss_stamp > 5 s` (`localization_health.hpp:193-205,230`). Repeated
  publications of one cached upstream status therefore keep it healthy forever;
  a backward ROS-time jump also produces a negative age which passes as fresh.
- The cached fix type and callback-time Fixed debounce feed `gps_is_fixed` and
  quality state used by the blade-on localization guard, precise/degraded
  navigation choice, undock calibration buffering, yaw seeding, and preflight.
- Concrete consequence: after physical GNSS observations stop, the mowing guard
  can fail to pause/stop and the tree can continue selecting Fixed-only behavior
  while only a cached sample is being republished.
- Existing tests cover a status topic that stops at a chosen time, but not a
  status topic that continues publishing the same receiver observation, nor ROS
  time pause/rewind.
- Remediation direction: update health and Fixed debounce only for a new,
  bounded-age observation; use a monotonic liveness clock and fail closed on
  invalid/future provenance.

Phase D resolution:

- The callback feeds public `position_observation_sequence` plus receiver
  receipt `header.stamp` into `PhysicalObservationTracker`. Only
  `kNewObservation`/`kSourceRestart` may replace RTK mode, accuracy, fix type,
  quality, or Fixed debounce state; cached/invalid/out-of-order deliveries are
  semantically inert.
- Before every behavior-tree tick, health requires both ROS receipt age and
  monotonic elapsed age since the last accepted identity to be within the
  existing `loc_gnss_stale_s` setting (5 s from bringup config/default).
  Expiry immediately clears Fixed/quality authorization and latches GNSS stale;
  the existing quality thresholds and recovery persistence are unchanged.
- ROS/steady rewind clears the identity epoch and Fixed debounce. Invalid/zero
  or future provenance cannot authorize; source restart applies only the new
  message's semantics, so an old Fixed value cannot resurrect by itself.
- Four behavior-specific regressions pin fresh Fixed, cached delivery without
  deadline extension, fail-closed expiry under the default persistence config,
  and genuine-observation recovery; all 16 historical health/latch cases stay
  green.

### MGNSS-004 — Dig-event GNSS trust follows cached status callback arrival

Status: **FIXED IN PHASE D**.

- The hardware bridge caches accuracy/Fixed state and assigns
  `dig_gnss_time_ = now()` on every `/gps/status` callback
  (`hardware_bridge_node.cpp:749-766`).
- The dig monitor calls GNSS fresh when `tick_now - dig_gnss_time_ < 2 s`
  (`hardware_bridge_node.cpp:2664-2771`), without observation identity,
  provenance age, or negative-age rejection. `DigTrustSigma` can therefore
  authorize an old GNSS anchor as long as status republication continues.
- The resulting verdict can hard-stop the robot, emit a dig event, and command
  a bounded reverse maneuver. Conversely, invalid stale anchoring can mask a
  true wheel/fused-pose discrepancy.
- Pure helper tests exercise a caller-supplied `gnss_fresh` boolean but do not
  verify how freshness is derived from repeated cached status or clock jumps.
- Remediation direction: derive trust from new-observation identity and bounded
  receipt age using monotonic liveness; make stale/future provenance explicitly
  untrusted before the physical-action state machine.

Phase D resolution:

- The hardware status callback now uses the same public sequence+receipt
  tracker and updates receiver accuracy/RTK eligibility only for accepted
  genuine evidence. The dig monitor evaluates the tracker, not callback time.
- The existing `dig_gnss_timeout_s` parameter remains the sole 2 s hardware
  GNSS trust deadline. No position/accuracy/RTK threshold, dig window, anchor,
  verdict, stop, event, or reverse behavior was changed.
- Cached Fixed delivery can remain transport-live while `DigTrustSigma`
  receives `fresh=false`/infinity; deterministic regression proves this cannot
  initialize a `DigDetectorState` anchor. A later advancing identity restores
  eligibility normally.
- Four new dig-freshness cases pass alongside all 27 historical dig/trust/
  escape cases.

### MGNSS-005 — UM980 dynamic-mode compatibility regressed across the pinned submodule upgrade

Status: **FIXED IN PHASE E**.

- Legacy commit `7343753` made UM980 Auto select `MODE ROVER UAV` after field
  evidence that `SURVEY MOW` could remain Float for hours while moving, and
  added a `--rover-dynamic-mode` override.
- The pinned baseline no longer contains that option and its Unicore profile
  emits `MODE ROVER SURVEY MOW` for supported rover models
  (`unicore_config_profile_builder.cpp:413-430`).
- MowgliNext still documents Auto as UM980 UAV, exposes Auto/UAV/Survey/Rover in
  the GUI, and both `start_gps.sh` and the Go plan/apply path conditionally pass
  the removed `--rover-dynamic-mode` argument. Auto passes no argument and gets
  Survey Mow; an explicit choice causes profile application to fail, while the
  startup script deliberately continues with the receiver's prior settings.
- Concrete consequence: the default can lose moving-platform RTK availability,
  while explicit user configuration is silently ineffective or leaves
  uncontrolled persistent receiver state.
- Remediation direction: restore an audited model-aware moving-rover policy in
  the current API, remove stale flags, make apply failure visible/fatal for the
  requested configuration, and test the emitted UM980 command plan.
- Phase E result: Universal GNSS now defaults UM980 to `MODE ROVER UAV`, retains
  Survey Mow for UM960/UM982/UB9A0 and the guarded generic Rover fallback, and
  accepts explicit `uav`, `survey_mow`, or `rover` overrides through both PLAN
  and APPLY. MowgliNext's existing GUI/startup selection is preserved and the
  Go and shell paths forward an override only when explicitly configured.
- Focused profile, planner, apply-equivalence, Go command, saved-config, and
  startup-order regressions all pass.

### MGNSS-006 — GPS-lock indication does not age receiver observation provenance

Status: **FIXED IN PHASE D**.

- The hardware bridge overwrites `gps_quality_` on each status callback and
  never ages or clears it (`hardware_bridge_node.cpp:749-766,947-954,2046-2068`).
- STM32 firmware lights the lock indicator whenever the received quality is at
  least 90 (`firmware/stm32/.../cpp_main.cpp:595-610`).
- Cached status republication keeps the LED current-looking after acquisition
  stops; if status delivery stops entirely, the last quality remains latched
  indefinitely in the high-level state.
- Concrete consequence: the physical operator indication can assert a GNSS lock
  after the receiver observation that justified it is stale or disconnected.
- Remediation direction: transmit an explicit age/valid bit or expire quality in
  the bridge, with boot, silence, reconnect, and cached-republication tests.

Phase D resolution:

- Firmware behavior was traced and left unchanged: `gps_quality < 90` turns
  `PANEL_LED_LOCK` off. The bridge therefore preserves every existing fresh
  quality/visual mapping and substitutes the established no-current value `0`
  whenever the shared hardware physical-observation verdict is stale.
- LED expiry reuses `dig_gnss_timeout_s` (2 s), the already-existing hardware
  GNSS status deadline; cached callbacks cannot extend it. The periodic
  high-level-state packet naturally propagates the cleared quality to STM32,
  while an accepted new observation restores the mapped quality.
- Four LED-specific regressions pin fresh Fixed/Float quality preservation,
  cached Fixed expiry, the exact safe output value, and genuine recovery. No
  firmware, protocol, color, brightness, or effect was changed.

### MGNSS-007 — Monitoring reports cached fix delivery as fresh observation health

Status: **FIXED IN PHASE D**.

- `mowgli_monitoring` subscribes to `/gps/fix`; `on_gps()` records callback
  `now()`, and `check_gps()` computes health from that callback age
  (`diagnostics_node.cpp:211-217,292-297,481-520`).
- While Universal GNSS republishes a still-runtime-fresh cached NavSatFix,
  monitoring delays warning/error as if new observations were arriving. A ROS
  clock rewind yields negative age and also passes.
- This masking is bounded by upstream's steady-clock runtime freshness when the
  receiver truly stops, but it still reports publication delivery rather than
  physical observation freshness and varies with acquisition/publication ratio.
- Remediation direction: monitor preserved observation identity/provenance and
  transport/runtime state separately, using monotonic deadlines.

Phase D resolution:

- Implementation-time launch revalidation identified the active production
  component as `mowgli_localization/localization_monitor_node`, subscribed to
  `/gps/absolute_pose`; the audit-era `mowgli_monitoring` source description is
  retained above as historical failure evidence but is not the launched path in
  this checkout.
- `AbsolutePose` has no position sequence, so the active monitor uses the
  shared tracker's receipt-only compatibility mode on its preserved
  `header.stamp`. Only an advancing accepted receipt may replace Fixed/Float/
  standalone flags. The existing `gps_timeout` remains 2 s and the existing 1 s
  mode debounce/quality classification remains intact.
- Monotonic physical-observation expiry now works even while ROS time is paused.
  Callback delivery liveness remains separately observable: transition logging
  distinguishes “cached delivery active, physical observations stale” from
  total delivery silence, without logging each cached callback.
- Twenty new monitoring/shared-policy cases cover its four required transitions
  plus startup, same-value/new identity, delayed duplicate, zero/future/rewound
  provenance, clock epochs, reconnect sequence reset, forward jump, both
  publication/acquisition-rate scenarios, and delivery/observation separation.

### MGNSS-008 — Backend/browser retain old typed GNSS state across stream failures

Status: confirmed from source; no disconnect/invalidation regression test exists.

- The Go `RosProvider` caches every latest logical-topic message and immediately
  replays it to subscribers (`gui/pkg/providers/ros.go:322-330,491-515`). It
  deletes the cache only when the final downstream listener unsubscribes, not
  when the Foxglove/ROS connection drops.
- `useWS` correctly surfaces multiplexed socket close/reconnect events through
  caller handlers (`gui/web/src/hooks/useWS.ts:126-136`).
- `useGnssStatus` supplies no-op error and info handlers and never clears or
  ages `gnssStatus` (`gui/web/src/hooks/useGnssStatus.ts:11-41`). A backend-to-ROS
  outage does not close the still-connected browser/backend stream, so the Go
  cache also remains the last typed sample.
- Concrete consequence: RTK Fixed, receiver identity, correction status, and
  accuracy can remain operator-visible and can continue satisfying onboarding
  readiness until a newer sample arrives. Reconnect can replay the stale cached
  value before fresh ROS state.
- Existing GNSS UI tests cover labels/field projection, not disconnect,
  invalidation, receipt age, or reconnect ordering.
- Remediation direction: carry a monotonic receive-age/validity state through
  the backend and hook, invalidate on either upstream or downstream connection
  loss, and require bounded receipt-provenance age for readiness.

### MGNSS-009 — Correction diagnostics cache outlives source/incarnation and collapses flow with health

Status: **FIXED in Phase F**.

- Both bridges store diagnostic entries indefinitely by name and never remove
  absent entries, age them, or tie them to a source incarnation
  (`universal_gnss_topic_bridge.cpp:360-385` and Python lines 285-300).
- The projection always prefers `universal_gnss_ntrip/rtcm_forwarding` over the
  receiver's forwarding diagnostic and similarly prefers the NTRIP MSM summary
  (`universal_gnss_topic_bridge.cpp:388-478`). A dead/restarted/disabled NTRIP
  node can therefore leave its prior `active` and station/MSM fields grafted
  onto every later receiver status indefinitely, shadowing live receiver data.
- The public status and onboarding correction check reduce the separate NTRIP
  transport/accepted-response/flow/semantic states to
  `correction_stream_status`; onboarding passes solely on `ACTIVE`
  (`gui/web/src/components/onboarding/readinessChecks.ts:97-118`) without
  requiring decoded/valid, non-empty MSM semantic health.
- The browser diagnostic merge ORs diagnostic-derived capability/value flags
  into each current typed sample. A stale cached diagnostic can consequently
  resurrect a correction value that the current core status explicitly cleared.
- Violated invariants: correction source ownership must reset on reconnect or
  source change, and valid correction flow/semantic health must not be inferred
  from generic transport/frame-forwarding activity.
- Concrete consequence: setup can report active corrections from an obsolete
  source or pass a stream carrying malformed/empty dynamic corrections.
- Existing bridge functional tests cover only one status mapping and one byte
  exact RTCM frame; no diagnostic removal, restart, source switch, station
  change, semantic-invalid-active, or expiry case exists.
- Remediation direction: make correction state an explicit typed, timestamped,
  source-owned contract; clear it on absence/restart/source change and require
  semantic dynamic-correction health separately from transport/forwarding.

Phase F resolution:

- Both bridges now hold at most one complete NtripNode snapshot and one complete
  ReceiverNode snapshot. Each receipt replaces that owner's correction entries;
  arrays for unrelated components are OMIT, while missing fields in a represented
  owner are CLEAR. Authority expires after 2 s on steady/monotonic time even if
  ROS time pauses.
- Upstream `hardware_id` is the source identity. A source change replaces the
  old epoch; conflicting/missing identity fails closed. Non-streaming lifecycle,
  reconnect, station mismatch, semantic invalidity, expiry, and a newer full
  snapshot clear incompatible state. Diagnostic ROS stamps are ordering evidence
  only, so an older SET cannot refresh lifetime or resurrect a newer CLEAR.
- ReceiverNode forwarding is authoritative for successful receiver writes.
  NtripNode forwarding is only the explicit compatibility fallback when no
  current ReceiverNode owner exists. Expired/disconnected NTRIP cannot shadow a
  current receiver forwarding/MSM observation; receiver fallback never invents
  NTRIP transport or semantic health.
- `GnssStatus.msg` appends typed transport, response-accepted, valid-flow, and
  semantic-health states plus correction, forwarding, and MSM source identities.
  Existing stream/MSM fields remain backward compatible. Capability/value masks
  expose current evidence independently for each group.
- Semantic `HEALTHY` requires upstream correction availability and parser health
  plus a same-snapshot valid base and current decoded/valid non-zero-cell MSM for
  the same station. Forwarding, connection, or response acceptance alone cannot
  satisfy it. Static 1005/1006 alone and malformed/zero-cell MSM fail closed;
  RTCM 1230 is not consulted and remains optional.
- The browser's correction-specific merge gives typed SET/CLEAR ownership over
  cached diagnostic fallback. Onboarding and the GPS correction alert require
  current source identity, streaming/accepted response, active valid flow,
  receiver forwarding, healthy semantics, and usable current MSM. RTK Fixed is
  deliberately not part of this correction-health criterion.
- Residual limitation: current diagnostics expose source/mountpoint identity and
  ROS ordering, not an independent process-incarnation token. A restart with the
  exact same identity and unusable/equal ROS ordering cannot be perfectly
  distinguished. Complete snapshots, lifecycle CLEAR, older-snapshot rejection,
  and monotonic expiry provide the strongest deterministic fail-closed policy
  available without changing the read-only upstream contract.

No independent defect was found in message-1230 handling: downstream readiness
does not require 1230, and the detailed diagnostics view presents its absence as
optional information rather than as a normal-RTK failure.

### MGNSS-010 — Field-proven Unicore correction-age policy was lost and exposed settings are not applied

Status: **FIXED IN PHASE E**.

- Legacy commit `a6d3cf3` changed the field policy to 120 s RTK and 300 s DGPS:
  its recorded rationale says a 10 s correction interruption repeatedly tears
  down ambiguity convergence, while retaining DGPS for 600 s leaves navigation
  on metre-class corrections for too long.
- The pinned current builder emits 10 s RTK and 600 s DGPS
  (`unicore_config_profile_builder.cpp:413-430`). This is not an equivalent
  implementation of the field-proven behavior.
- MowgliNext's GUI/schema still exposes `gnss_unicore_rtk_timeout_s` and
  `gnss_unicore_dgps_timeout_s`, but the saved-config parser, Go build/apply plan,
  and `start_gps.sh` do not consume or forward either value. The controls are
  inert regardless of what the user saves.
- Concrete consequence: short ordinary correction gaps can force a full RTK
  reconvergence, while prolonged gaps can retain DGPS well beyond the intended
  degradation window; operators cannot apply the exposed mitigation.
- Remediation direction: establish one documented default policy, wire both
  values end to end with range/unit validation, expose the effective receiver
  command, and regression-test default and overridden plans.
- Phase E result: the Unicore rover profile emits RTK 120 s and DGPS 300 s by
  default, retains `CONFIG RTK RELIABILITY 3 1`, and accepts optional independent
  RTK/DGPS overrides in the documented `1..1800` second range.
- Schema/UI bounds, settings normalization, saved-config Go PLAN/APPLY commands,
  startup dry-run behavior, Universal GNSS plan/apply equivalence, defaults,
  overrides, and rejection boundaries are covered by passing regressions.

### MGNSS-011 — Supported 1 Hz fixes can never produce COG heading

Status: **FIXED in Phase C**.

- Original defect: `cog_to_imu` used a fixed 0.50 s maximum receipt delta even
  though MowgliNext exposes physical receiver profiles at 1, 5, 7, and 10 Hz.
  Every genuine 1 Hz sample therefore discarded the previous anchor and could
  never form a moving heading baseline.
- Phase C reuses the existing `gnss_profile_rate_hz` robot setting—the same
  value passed to Universal GNSS profile application—as the sole physical-rate
  source. `navigation.launch.py` passes it to the node under the explicit name
  `physical_gnss_observation_rate_hz`; no ROS publication-rate input reaches
  the COG timing policy.
- The automatic `max_sample_dt_s` default is
  `expected_period * 1.5`. The 50% tolerance admits normal receipt jitter at
  every supported rate while a missed complete next cycle (two periods) still
  resets. The pre-existing `max_sample_dt_s` operator override remains
  available but must be finite and at least one configured physical period;
  `min_sample_dt_s = 0.05` remains unchanged.
- A production `CogObservationTiming` gate uses integer receipt nanoseconds
  only. It has a bounded 64-receipt history so current and delayed cached
  republications are ignored without disturbing the baseline, while same
  coordinates with a new receipt remain genuine. No callback, publication, or
  delivery interval participates in displacement timing.
- Zero, future, older-than-2 s, unseen-rewound, and old-ROS-epoch provenance
  fails closed. An excessive but valid forward receipt gap accepts the current
  sample only as a clean seed. Receiver reconnect therefore cannot join motion
  across an unbounded gap; the next normal sample can form a new baseline.
- Stationary yaw now stores receipt provenance separately from monotonic latch
  time. Negative ROS age or ROS/steady rewind clears it; either ROS provenance
  age or monotonic liveness age exceeding `stationary_max_age_s` expires it.
  Drift covariance uses the larger valid elapsed age, so paused ROS time cannot
  preserve a latch indefinitely.
- Output topic, frame, heading convention, covariance calculation, displacement
  threshold, wheel-speed requirement, rotation/sweep gates, and moving versus
  stationary behavior are unchanged.
- Regression coverage proves moving heading-capable baselines at 1/5/7/10 Hz;
  cached 10 Hz delivery around a physical 1 Hz source; same-coordinate new
  observations; 50% jitter and preserved minimum interval; excessive gap
  reseed; delayed duplicate; receipt/ROS rewind; zero/future/large-forward
  stamps; reconnect; publication/acquisition independence; invalid config; and
  stationary negative-age, rewind, ROS-pause, and monotonic expiry behavior.
- Residual operational risk: `gnss_profile_rate_hz` is the requested/expected
  physical rate. Receiver-profile application is intentionally non-fatal, so a
  receiver that rejects configuration may retain a different effective rate.
  The COG gate then fails safely by reseeding rather than joining excessive
  gaps, but heading availability requires correcting the receiver configuration.

## Test coverage gaps

Required regression additions:

- Cached status/fix republication versus new acquisition is now covered for
  behavior, dig trust, the hardware LED, localization monitoring, FusionGraph,
  NavSat projection, and COG. Backend/GUI disconnect retention remains open as
  MGNSS-008.
- Fixed -> Float -> no-fix semantic mappings remain covered by existing shared
  adapter/consumer tests; Phase B covers adversarial cross-topic ordering and
  Phase D prevents cached state from changing or extending consumer authority.
- Explicit SET/OMIT/CLEAR invalidation of accuracy, DOP, heading, baseline, and
  satellite fields through the bridge and every cache.
- ROS zero/future provenance, pause via monotonic expiry, backward/forward jump,
  negative age, receipt rewind, and reconnect epoch behavior are deterministic
  Phase D regressions for the shared behavior/hardware/monitoring policy.
  End-to-end rosbag replay remains hardware validation rather than a unit gap.
- Unique GNSS queue backlog and late fixed messages after their graph nodes have
  expired. Phase A covers delayed duplicates and the existing historical-node
  timestamp suite remains green.
- COG output at every supported acquisition rate, independently varied ROS
  publication rate, cached duplicate stamps, and stationary-latch clock jumps
  is covered by Phase C.
- NTRIP connected without valid RTCM, malformed/zero-cell MSM, static-only data,
  caster reconnect, mountpoint/source change, station change, stale diagnostic
  removal, old-source resurrection, SET/OMIT/CLEAR, and paused/rewound ROS time
  are covered by Phase F C++/Python/browser regressions.
- Backend/Foxglove and browser WebSocket loss/reconnect invalidation.
- UM980 Auto and explicit dynamic modes against the pinned tools.

## Infrastructure issues excluded from product findings

The following recovered baseline issues are build/test infrastructure concerns,
not GNSS product findings. The quota-limited final pass did not rerun them, and
none is used as proof for the 11 findings:

- normal workspace synchronization may omit `sensors/gps/mowgli_gnss_bridge`;
- installed Fields2Cover 2.0 versus `mowgli_coverage` 3.0 requirement;
- bridge license/style linter failures;
- path-sensitive bringup ownership test;
- stale installer compose-variable assertion.

## Scoped build and test results

Completed results (all commands ran as non-root user `ubuntu`, with build
artifacts under `/tmp`):

| Scope | Result | Audit interpretation |
|---|---|---|
| Pinned Universal GNSS standalone CMake/CTest suite | **PASS — 61/61** | Confirms the audited upstream core/protocol/driver/transport/NTRIP/tool baseline, including current Unicore profile and framed transport behavior. It does not exercise MowgliNext's public bridge or consumers. |
| `universal_gnss_ros2` isolated colcon build | **PASS — 1/1 package** | Confirms the exact pinned ROS adapter/node sources build against ROS 2 Kilted. Only CMake deprecation warnings were emitted. |
| `universal_gnss_ros2` isolated colcon tests | **PASS — 93/93 assertions, 0 failures/errors/skips** | Six test executables passed, including receiver freshness/sequence behavior, NTRIP cached-status rejection, diagnostic projection, replay, and adapters. This reinforces that the audited contract exists upstream. |
| Phase A isolated colcon build: `mowgli_interfaces`, `fusion_graph`, `mowgli_gnss_bridge` | **PASS — 3/3 packages** | Confirms the public message addition, both bridge implementations, shared freshness policy, and FusionGraph integration compile together. Only pre-existing CMake/Eigen/NodeOptions warnings were emitted. |
| Phase A FusionGraph GNSS tests | **PASS — 11/11 tests** | Existing historical-timestamp tests (6) plus new observation-identity/freshness tests (5): cached receipt, next receipt, delayed duplicate, sequence restart, receiver silence, future/negative age, and ROS rewind. |
| Phase A bridge tests | **PASS — 5/5 tests** | C++ tests (3) and Python tests (2) agree on exact stamp/sequence propagation, cached sequence reuse, and a new sequence for an identical genuine position. |
| Phase B isolated `mowgli_localization` build | **PASS — 1/1 package** | The production association library, projection node, and both focused test executables compile and install against the Phase A underlay. Only existing ament target-dependency deprecation warnings were emitted. |
| Phase B focused localization tests | **PASS — 21/21 tests** | Eighteen association regressions cover all required pairing/order/cache/duplicate/transition/timeout/capacity/restart/clock/provenance cases; three existing projection/typed-covariance tests remain green. |
| Phase B NavSat-only launch integration | **PASS — 2/2 functional tests plus clean shutdown** | A fresh fix with no local typed-status publisher emits exactly one `/gps/absolute_pose`, emits no `/gps/pose_cov`, and does not create a `/gps/status` publisher. |
| Phase C isolated `mowgli_localization` build | **PASS — 1/1 package** | The COG node and receipt/latch timing policy compile and install against the committed Phase A/B tree. Only existing ament target-dependency deprecation warnings were emitted. |
| Phase C focused COG tests | **PASS — 34/34 cases** | Nineteen timing/latch regressions cover every required rate, identity, cadence, gap, rewind, future, restart, and latch case; all 15 pre-existing heading/sweep/latch-rotation tests remain green. |
| Phase C launch wiring tests | **PASS — 7/7** | The new AST guard proves `gnss_profile_rate_hz` reaches `cog_to_imu.physical_gnss_observation_rate_hz` as a bare value; all six existing launch-injection guards remain green. |
| Phase D isolated build: `mowgli_interfaces`, `mowgli_behavior`, `mowgli_hardware`, `mowgli_localization` | **PASS — 4/4 packages** | Shared physical-observation freshness and all three production consumers compile/install together. Only pre-existing ament dependency deprecation and unrelated BehaviorTree nodiscard warnings were emitted on the clean build. |
| Phase D focused consumer tests | **PASS — 75/75 cases** | `test_localization_health` 20/20, `test_dig_detector` 35/35, and `test_localization_monitor_freshness` 20/20. Thirty-two are new Phase D cases (4 behavior, 4 dig, 4 LED, 20 monitoring/shared cross-cutting); 43 historical cases remain green. |
| Phase D complete affected-package GTest suites | **PASS — 432/432 cases** | Behavior 198, hardware 111, localization 123; no failures/errors/skips. The complete CTest/lint pass exposed one cpplint complaint about anonymous test-parameter syntax, which was corrected; the affected health target and cpplint both passed on rerun. |
| Phase F isolated build: pinned `universal_gnss`, `mowgli_interfaces`, `mowgli_gnss_bridge` | **PASS — 3/3 packages** | The pinned read-only ROS2 dependency, append-only public interface, generated types, tracker, and production bridge compile together under `/tmp`. |
| Phase F C++ bridge tests | **PASS — 9/9 cases** | Six deterministic tracker cases cover the correction matrix; all three existing end-to-end mapping/sequence/RTCM tests remain green. |
| Phase F Python bridge tests | **PASS — 7/7 cases** | The fallback mirrors the C++ source/lifetime/semantic vectors plus existing observation identity behavior. |
| Phase F focused frontend tests | **PASS — 75/75 cases** | Projection CLEAR ownership, semantic readiness failures, existing Fixed/Float/no-fix mappings, and the typed diagnostics UI pass. |
| Phase F TypeScript and Go validation | **PASS** | `tsc --noEmit` passes; generated Go message/provider suites pass, including `pkg/msgs/mowgli` and `pkg/providers`. |

The remaining downstream ROS packages, frontend, Go provider, and installer
matrices were not rerun after the user's quota-prioritization instruction. The
recovered pre-audit baseline reported installer GNSS matrix 18/18, frontend GNSS
tests 57/57, and Go provider tests green; those historical results are not
treated as fresh validation and do not cover the adversarial gaps above.

## Adversarial scenario coverage

Static audit coverage is complete for cached republication,
observations stopping while typed status continues, Fixed/Float transition
skew, delayed/backlogged messages, stream error retention, NTRIP source restart,
semantic-invalid active RTCM, legacy UM980 configuration mismatch, and the
supported 1 Hz COG dead zone.

For NavSat projection specifically, Phase B converts the static proof into
deterministic production-path regression coverage for callback reordering,
backlog, bounded timeout/capacity, duplicate/cache delivery, sequence restart,
receipt/clock discontinuity, and ambiguous identity.

For COG specifically, Phase C converts the 1 Hz dead-zone proof into
deterministic production-gate coverage for every exposed receiver rate,
publication/acquisition decoupling, bounded gaps/history, reconnect, provenance
and clock discontinuity, and stationary-latch expiry.

For the four Phase D consumers, deterministic coverage now proves that callback
delivery and physical freshness diverge correctly: 1 Hz acquisition with 10 Hz
callbacks follows the advancing identity; sparse 1 Hz publication of a faster
source does not infer cadence; cached/delayed messages cannot refresh; receipt
and clock epochs fail closed; and new accepted evidence restores the existing
Fixed/Float/no-fix, dig, LED, and monitoring semantics.

For correction projection, Phase F converts the source-lifetime proof into
deterministic C++/Python parity coverage for complete snapshots, monotonic
expiry, paused/rewound ROS time, disconnect/reconnect, source and station
changes, old-source rejection, receiver fallback, semantic-invalid active
forwarding, static-only and zero-cell data, optional 1230, and SET/OMIT/CLEAR.
Frontend regressions prove that typed CLEAR survives diagnostic fallback and
readiness requires the distinct current semantic dimensions.

The GPS dock detector was explicitly reviewed and is not counted as stale-GNSS
retention: after a Fixed solution it converts the dock to the continuous odom
frame and intentionally republishes that dead-reckoned landmark through later
Float/no-fix periods. It publishes nothing before the first Fixed anchor.

## Reviewed non-findings

- Core public SET/OMIT/CLEAR propagation is correct: each bridge output is newly
  constructed and copies capability and value masks. Only the diagnostic
  correction merge in MGNSS-009 reintroduces cleared state.
- NavSat covariance units are consistent: upstream converts reported sigmas to
  variances, and downstream derives horizontal sigma from covariance before
  thresholding. ENU use is consistently x=east, y=north.
- No production health/readiness consumer was found that makes RTCM 1230
  mandatory. Its optional display is not a defect.
- The current bounded pending-frame FIFO preserves a complete accepted RTCM
  frame over partial/EAGAIN writes. It is equivalent or superior to legacy
  commit `4bf5d0a`; no downstream regression remains for that old change.
- The GPS dock detector's continued publication after loss of Fixed is an
  intentional odom-frame dead-reckoned landmark, not cached GNSS masquerading as
  a new observation.

## Hardware validation plan

Hardware is not required to close this software/static audit. After fixes,
validate both Unicore/UM982-family and u-blox F9P receivers on a PC and on the
robot. For each receiver, record physical output, internal Universal GNSS status,
public status/fix, FusionGraph factor acceptance/mode gates, behavior guard,
hardware LED/dig trust, monitoring, backend, and GUI side by side while testing:

- high receiver rate with low ROS publication rate;
- low receiver rate with high ROS publication rate;
- repeated identical genuine positions versus cached republication;
- receiver silence, disconnect/reconnect, and identical-fix recovery;
- Fixed/Float/standalone/no-fix transitions in both directions;
- correction loss/resume, connected-without-valid-RTCM, caster reconnect, and
  mountpoint/station/base changes;
- explicit runtime invalidation of accuracy, DOP, heading, baseline, and
  satellite state;
- receipt timestamps, queue delay, ROS time pause/jumps, and consumer agreement.

## Recommended fix order

Final dependency order:

1. **Phases A through D complete:** public observation identity, FusionGraph,
   bounded NavSat/status association, COG timing, behavior health, dig trust,
   the lock LED, and localization monitoring are provenance-aware and tested.
2. **Phase E complete:** MGNSS-005 and MGNSS-010 are fixed together at receiver
   configuration boundaries with model-aware rover policy and effective,
   validated correction-age settings.
3. **Phase F complete:** correction semantic health, expiry, source/station
   ownership, browser CLEAR behavior, and readiness are explicit and tested.
4. Fix the remaining MGNSS-008 by carrying bounded validity and disconnect
   invalidation through the generic backend/browser lifecycle.

Findings sharing the observation-identity/public-contract root should be fixed
and regression-tested together even where this report retains separate IDs for
independent consumer consequences.

## Audit-era repository cross-check

Before Phase A, the completed audit was on branch `audit/gnss-downstream` at
top-level HEAD `bc8b1de3`, with Universal GNSS clean at
`45fe44a031520f9dfe4bfc07fd515952d1a1ea88` and only the audit report present as
an audit deliverable. Phase A intentionally changes the narrow production and
test files recorded above; no submodule content, commit, or remote is changed.

## Phase A final repository cross-check

- Branch: `audit/gnss-downstream`.
- Top-level HEAD: `880dd4f28cc3`; Codex created no commit and performed no push.
- Universal GNSS gitlink/HEAD: exact required
  `45fe44a031520f9dfe4bfc07fd515952d1a1ea88`, clean (leading blank marker from
  `git submodule status`). The unrelated `opennav_coverage` submodule is also at
  its recorded clean gitlink.
- `git diff --check`: **PASS**.
- ROS2 C++ formatting: `clang-format -n -Werror
  -style=file:ros2/.clang-format` and the repository-preferred
  `git clang-format --diff origin/main` check both **PASS** for every touched
  ROS2 C++ source/header. The bridge C++ files retain their pre-existing local
  style, leaving a one-line production change plus focused test additions.
- `git diff --stat`: 13 tracked files changed, 352 insertions, 144 deletions.
  Git excludes the three untracked new files (380 lines total) from this
  statistic.
- `git status --short`: 13 tracked modified files and three untracked new files,
  exactly (the tracked paths also match `git diff --name-status`):

```text
 M MOWGLINEXT_GNSS_AUDIT.md
 M ros2/src/fusion_graph/CMakeLists.txt
 M ros2/src/fusion_graph/include/fusion_graph/fusion_graph_node.hpp
 M ros2/src/fusion_graph/src/fusion_graph_node_callbacks_a.cpp
 M ros2/src/fusion_graph/src/fusion_graph_node_callbacks_b.cpp
 M ros2/src/fusion_graph/src/fusion_graph_node_setup_comms.cpp
 M ros2/src/fusion_graph/src/fusion_graph_node_timer.cpp
 M ros2/src/mowgli_interfaces/msg/GnssStatus.msg
 M sensors/gps/mowgli_gnss_bridge/CMakeLists.txt
 M sensors/gps/mowgli_gnss_bridge/package.xml
 M sensors/gps/mowgli_gnss_bridge/src/universal_gnss_topic_bridge.cpp
 M sensors/gps/mowgli_gnss_bridge/test/test_universal_gnss_topic_bridge.cpp
 M sensors/gps/universal_gnss_topic_bridge.py
?? ros2/src/fusion_graph/test/test_gnss_observation_freshness.cpp
?? ros2/src/mowgli_interfaces/include/mowgli_interfaces/gnss_observation_freshness.hpp
?? sensors/gps/mowgli_gnss_bridge/test/test_python_bridge.py
```

All changed and new files are owned by the normal project user `ubuntu`; build
artifacts remain under `/tmp`. Universal GNSS content, forbidden Phase A scopes,
and remotes were not modified.

## Phase B final repository cross-check

- Branch: `audit/gnss-downstream`.
- Top-level HEAD: `93475cdc fix(gnss): preserve observation identity
  downstream`; Phase A is present. Phase B created no commit and performed no
  push.
- Universal GNSS gitlink/HEAD: exact required
  `45fe44a031520f9dfe4bfc07fd515952d1a1ea88`, with an empty submodule
  `git status --short`. No Universal GNSS content or gitlink changed.
- `git diff --check`: **PASS**.
- ROS2 C++ formatting: both `clang-format -n -Werror
  -style=file:ros2/.clang-format` and `git clang-format --diff origin/main`
  report no changes for every touched C++ source/header.
- Focused validation: the `mowgli_localization` targets rebuild; all 21 focused
  C++ test cases pass; the two functional NavSat-only launch assertions and
  clean process-shutdown assertion pass.
- `git status --short`, `git diff --stat`, `git diff --name-only`, and the
  untracked-file listing show only the eight intended Phase B paths: five
  tracked modifications and three new files:

```text
 M MOWGLINEXT_GNSS_AUDIT.md
 M ros2/src/mowgli_bringup/test/test_navsat_status_universal.launch.py
 M ros2/src/mowgli_localization/CMakeLists.txt
 M ros2/src/mowgli_localization/include/mowgli_localization/navsat_to_absolute_pose_node.hpp
 M ros2/src/mowgli_localization/src/navsat_to_absolute_pose_node.cpp
?? ros2/src/mowgli_localization/include/mowgli_localization/navsat_status_association.hpp
?? ros2/src/mowgli_localization/src/navsat_status_association.cpp
?? ros2/src/mowgli_localization/test/test_navsat_status_association.cpp
```

All eight paths are owned by the normal project user `ubuntu`; isolated build
artifacts remain under `/tmp`. MGNSS-003 through MGNSS-011, remotes, and
submodules were not modified.

## Phase C final repository cross-check

- Branch: `audit/gnss-downstream`.
- Top-level HEAD: `1c586082 fix(gnss): pair navsat fixes with matching
  status`; committed Phases A and B are present. Phase C created no commit and
  performed no push.
- Universal GNSS gitlink/HEAD: exact required
  `45fe44a031520f9dfe4bfc07fd515952d1a1ea88`, with an empty submodule
  `git status --short`. No Universal GNSS content or gitlink changed.
- `git diff --check`: **PASS**.
- ROS2 C++ formatting: both `clang-format -n -Werror
  -style=file:ros2/.clang-format` and `git clang-format --diff origin/main`
  report no changes for every touched C++ source/header.
- Focused validation: isolated `mowgli_localization` build **PASS**; 19/19 new
  receipt/latch tests and 15/15 historical COG tests **PASS**; 7/7 launch
  injection tests **PASS**.
- `git status --short`, `git diff --stat`, `git diff --name-only`, and the
  untracked-file listing show only the seven intended Phase C paths: five
  tracked modifications and two new files:

```text
 M MOWGLINEXT_GNSS_AUDIT.md
 M ros2/src/mowgli_bringup/launch/navigation.launch.py
 M ros2/src/mowgli_bringup/test/test_launch_injection.py
 M ros2/src/mowgli_localization/CMakeLists.txt
 M ros2/src/mowgli_localization/src/cog_to_imu_node.cpp
?? ros2/src/mowgli_localization/include/mowgli_localization/cog_observation_timing.hpp
?? ros2/src/mowgli_localization/test/test_cog_observation_timing.cpp
```

All seven paths are owned by the normal project user `ubuntu`; isolated build
artifacts remain under `/tmp`. MGNSS-003 through MGNSS-010, remotes, and
submodules were not modified.

## Phase D final repository cross-check

- Branch: `audit/gnss-downstream`.
- Top-level HEAD: `97c75ebeea5344e303d37e2ed5f2d1ad7fedca45`
  (`fix(gnss): align COG timing with physical observation rate`); committed
  Phases A-C are present. Phase D created no commit and performed no push.
- Universal GNSS gitlink/HEAD remains the exact required
  `45fe44a031520f9dfe4bfc07fd515952d1a1ea88`; `git submodule status` has the
  clean leading blank marker and submodule `git status --short` is empty. No
  Universal GNSS content or gitlink changed.
- `git diff --check`: **PASS**.
- ROS2 C++ formatting: `clang-format -n -Werror
  -style=file:ros2/.clang-format` and repository-preferred
  `git clang-format --diff origin/main` both report no changes for every Phase D
  C++ source/header.
- Isolated non-root validation under `/tmp`: four-package build **PASS**;
  75/75 focused cases **PASS** (32 new); all 432 GTest cases in the three
  affected packages **PASS**. The only newly exposed lint issue was test-only
  anonymous-parameter syntax; after correction both the affected 20-case test
  and cpplint passed.
- `git status --short`, `git diff --stat`, `git diff --name-only`, and
  `git diff --name-status` show 12 intended tracked modifications plus three
  intended new files. As expected, ordinary Git diff/stat output excludes the
  untracked files until they are added:

```text
 M MOWGLINEXT_GNSS_AUDIT.md
 M ros2/src/mowgli_behavior/CMakeLists.txt
 M ros2/src/mowgli_behavior/include/mowgli_behavior/localization_health.hpp
 M ros2/src/mowgli_behavior/src/behavior_tree_node.cpp
 M ros2/src/mowgli_behavior/test/test_localization_health.cpp
 M ros2/src/mowgli_hardware/CMakeLists.txt
 M ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp
 M ros2/src/mowgli_hardware/test/test_dig_detector.cpp
 M ros2/src/mowgli_interfaces/include/mowgli_interfaces/gnss_observation_freshness.hpp
 M ros2/src/mowgli_localization/CMakeLists.txt
 M ros2/src/mowgli_localization/include/mowgli_localization/localization_monitor_node.hpp
 M ros2/src/mowgli_localization/src/localization_monitor_node.cpp
?? ros2/src/mowgli_hardware/include/mowgli_hardware/gnss_hardware_status.hpp
?? ros2/src/mowgli_localization/include/mowgli_localization/localization_monitor_policy.hpp
?? ros2/src/mowgli_localization/test/test_localization_monitor_freshness.cpp
```

All 15 paths are owned by `ubuntu`; build/test/log artifacts remain isolated
under `/tmp`. Firmware, Universal GNSS, MGNSS-005/008/009/010 scopes, remotes,
and submodules were not modified.

## Phase F final repository cross-check

- Branch: `audit/gnss-downstream`; top-level HEAD remains exact required
  `c6cd8b907bb43bb42611976e51e39cb2deca45c9`, with
  `origin/dev...HEAD` still `0 7`. Phase F created no commit and performed no
  push.
- Universal GNSS gitlink/worktree HEAD remains exact required
  `5281472116669972ae12b9d1997d66b064671cf5`, and its `git status --short` is
  empty. No Universal GNSS content or gitlink changed.
- `git diff --check`: **PASS** for tracked changes; explicit `--no-index
  --check` passes for all three new C++ files. The new tracker/header/test are
  clang-format clean under the sensor bridge's existing Google-style layout.
- Non-root validation under `/tmp`: pinned dependency/interface/bridge builds
  **PASS**; C++ 9/9, Python 7/7, focused frontend 75/75, TypeScript no-emit, Go
  message/provider tests, and reproducible TypeScript/Go message generation all
  **PASS**.
- The scoped worktree contains 19 tracked modified paths plus three new Phase F
  tracker paths (883 new-file lines). Tracked diff is 1,180 insertions and 359
  deletions. The unrelated untracked
  `.devcontainer/devcontainer-lock.json` remains untouched and excluded.
- All sampled changed/new paths are owned by `ubuntu`; build/test/log artifacts
  remain under `/tmp`. Generic backend/Foxglove/browser disconnect lifetime,
  MGNSS-008, remotes, submodules, and unrelated production paths were not
  modified.

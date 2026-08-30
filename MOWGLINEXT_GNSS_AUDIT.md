# MowgliNext GNSS Downstream Audit

> Final audit report and remediation ledger. Phase A and Phase B implementation
> status is recorded separately from the original findings so open consumers
> are not accidentally presented as fixed.

## Executive summary

Audit status: **complete**.

The hard pre-flight passed: the top-level checkout was clean on
`audit/gnss-downstream`, and the Universal GNSS submodule was clean and pinned
to `45fe44a031520f9dfe4bfc07fd515952d1a1ea88` before Phase A edits began.
Phase B started from clean HEAD `93475cdc`, containing the committed Phase A
changes, with the same exact clean Universal GNSS baseline.

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

Phases A and B change production code only for the public contract, both
bridges, FusionGraph deduplication/freshness, the shared freshness primitive,
and the NavSat projection association targeted by MGNSS-002.

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
  -> cog_to_imu                           (/gps/fix -> /imu/cog_heading)
  -> navsat_to_absolute_pose              (exact bounded receipt-paired fix/status)
       -> /gps/absolute_pose
       -> /gps/pose_cov
  -> behavior/localization-health         (/gps/status)
  -> hardware bridge                      (/gps/status; LED and dig trust)
  -> monitoring                           (/gps/fix)
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
| Receipt provenance stamp | `GnssStatus.stamp` and `NavSatFix.header.stamp` are local receiver acceptance/receipt time in ROS-clock domain, not publication or callback time. | Bridge preserves the stamp in public `header.stamp`; some consumers ignore it and record callback `now()`. | violated by multiple consumers |
| Position observation identity | Sequence advances only for an accepted position observation, including numerically identical observations; cached publication keeps the sequence. | Public status appends the sequence and both bridges copy it verbatim. FusionGraph deduplicates its fix-only path by receipt; NavSat projection uses the exact receipt pair plus sequence for cache/ambiguity/restart handling. Other status consumers have not yet adopted the sequence. | boundary, FusionGraph, and NavSat projection fixed; consumer adoption incomplete |
| Acquisition vs publication | Physical acquisition is independent of `publish_rate_hz`; fresh cached state may be republished. | FusionGraph ignores receipt-identical republications before factor/latch effects. NavSat projection cannot turn cached status into a new pair. Behavior, hardware, and monitoring still derive freshness from delivery callbacks. | FusionGraph/projection fixed; other consumers open |
| SET / OMIT / CLEAR | Capability and value masks preserve supported/current-value tri-state semantics. | The bridge reconstructs each core message and copies both masks, so core clears survive. The GUI diagnostic merge ORs old diagnostic-derived correction flags/values back into the current sample. | core preserved; correction projection violated |
| Clock domains | Upstream liveness uses steady time; public provenance uses ROS time. | Shared policy separates ROS provenance age from monotonic delivery and rejects negative age; FusionGraph and NavSat projection use it. Projection deadlines/capacity are monotonic and ROS/steady rewinds reset its association epoch. Other audited consumers have not adopted it. | policy/FusionGraph/projection fixed; other consumers open |
| Correction states | Transport, accepted response, valid flow, semantic health, and receiver RTK solution are distinct. | Public projection derives one stream enum from a cached forwarding diagnostic; setup readiness passes on `ACTIVE` without semantic validity. | violated |
| RTCM semantics | 1005/1006 are static-base data, MSM is dynamic correction data, and 1230 is optional for normal RTK health. | The UI shows 1230 availability only as detail; no downstream health/readiness gate was found that requires it. Dynamic correction health is nevertheless collapsed with forwarding activity. | 1230 preserved; flow/health violated |
| Source/station ownership | Reconnect or source/station change must invalidate incompatible stale state. | Bridge diagnostics entries have no source incarnation or expiry and prefer old NTRIP entries over receiver entries. | violated |

## Old-fork compatibility delta

Final classification:

| Legacy behavior/change | Audited baseline | MowgliNext dependency | Classification |
|---|---|---|---|
| `--rover-dynamic-mode`; UM980 default `MODE ROVER UAV` | Removed; current Unicore rover profile emits `MODE ROVER SURVEY MOW`. | Startup script and GUI plan/apply still conditionally pass the removed flag; GUI still exposes Auto/UAV/Survey/Rover and describes UAV as the UM980 default. | **DOWNSTREAM DEPENDENCY / CONFIRMED REGRESSION** |
| Unicore correction-age windows (legacy RTK 120 s, DGPS 300 s) | Current profile uses RTK 10 s, DGPS 600 s. | MowgliNext exposes timeout fields, but the saved-config parser, Go plan/apply path, and startup script do not read or pass them. | **DOWNSTREAM DEPENDENCY / CONFIRMED REGRESSION** |
| Whole-frame RTCM forwarding | Current baseline has a bounded pending-frame FIFO and retries partial/EAGAIN writes without truncating the accepted frame. | No downstream dependency found. | **EQUIVALENT IN NEW BASELINE** |

## Findings summary

Original audit count: **11 findings: 0 P0, 6 P1, 5 P2, 0 P3**.
After Phase B: **2 fixed, 9 open**.

P1 denotes a high-impact loss of a primary localization/safety function or a
configuration regression with field-proven RTK consequences. P2 denotes a
bounded or secondary-path localization error, misleading health/operator state,
or readiness error without the same immediate primary-path impact.

| ID | Title | Severity | Classification | Evidence state |
|---|---|---:|---|---|
| MGNSS-001 | Observation identity is lost, so FusionGraph reuses cached fixes as fresh factors/evidence | P1 | **fixed in Phase A** | public bridge + receipt-only FusionGraph regressions |
| MGNSS-002 | NavSat projection pairs each fix with an unbounded, asynchronously cached typed status | P2 | **fixed in Phase B** | exact bounded association + 18 focused regressions + NavSat-only launch regression |
| MGNSS-003 | Behavior localization-health freshness follows cached status callback arrival | P1 | confirmed bug; Phase A prerequisite available | consumer remains callback-time based |
| MGNSS-004 | Dig-event GNSS trust follows cached status callback arrival | P1 | confirmed bug; Phase A prerequisite available | consumer remains callback-time based |
| MGNSS-005 | UM980 dynamic-mode compatibility regressed across the pinned submodule upgrade | P1 | confirmed integration regression | exact legacy/current/API/startup delta |
| MGNSS-006 | GPS-lock indication does not age receiver observation provenance | P2 | confirmed bug; Phase A prerequisite available | consumer expiry remains open |
| MGNSS-007 | Monitoring reports cached fix delivery as fresh observation health | P2 | confirmed bug; Phase A prerequisite available | consumer remains delivery-time based |
| MGNSS-008 | Backend/browser retain old typed GNSS state across stream failures | P2 | confirmed bug | backend cache and frontend lifecycle proof |
| MGNSS-009 | Correction diagnostics cache outlives source/incarnation and collapses flow with health | P2 | confirmed integration bug | bridge, upstream state, and readiness proof |
| MGNSS-010 | Field-proven Unicore correction-age policy was lost and exposed settings are not applied | P1 | confirmed integration regression | exact legacy/current/default and inert-setting delta |
| MGNSS-011 | Supported 1 Hz fixes can never produce COG heading | P1 | confirmed integration bug | interval/state-machine proof against supported profile |

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

Status: **OPEN; enabled for the next phase by the public sequence and shared
freshness policy**.

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

### MGNSS-004 — Dig-event GNSS trust follows cached status callback arrival

Status: **OPEN; enabled for the next phase by the public sequence and shared
freshness policy**.

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

### MGNSS-005 — UM980 dynamic-mode compatibility regressed across the pinned submodule upgrade

Status: confirmed against the legacy fork and current integration.

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

### MGNSS-006 — GPS-lock indication does not age receiver observation provenance

Status: **OPEN; enabled for the next phase by the public sequence and shared
freshness policy**.

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

### MGNSS-007 — Monitoring reports cached fix delivery as fresh observation health

Status: **OPEN; enabled for the next phase by the public sequence and shared
freshness policy**.

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

Status: confirmed across upstream diagnostics, both bridges, and GUI readiness.

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

No independent defect was found in message-1230 handling: downstream readiness
does not require 1230, and the detailed diagnostics view presents its absence as
optional information rather than as a normal-RTK failure.

### MGNSS-010 — Field-proven Unicore correction-age policy was lost and exposed settings are not applied

Status: confirmed against the legacy fork and current integration.

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

### MGNSS-011 — Supported 1 Hz fixes can never produce COG heading

Status: independently discovered and confirmed from the estimator state machine
and the supported-rate configuration.

- `cog_to_imu_node` subscribes to `/gps/fix` and defaults to
  `min_dt = 0.05 s`, `max_dt = 0.50 s`
  (`cog_to_imu_node.cpp:143-146,195-202`).
- For every accepted fix it computes the delta between preserved receipt stamps.
  If the delta exceeds `max_dt`, it clears both anchor and previous sample; the
  same callback then only seeds a new anchor and returns
  (`cog_to_imu_node.cpp:355-375,448-459`).
- The supported GNSS configuration explicitly offers a 1 Hz profile. Consecutive
  genuine observations are then about 1 s apart, so every observation resets
  and reseeds the estimator. No displacement pair survives and no moving
  `/imu/cog_heading` sample can be emitted.
- Identical cached republications are correctly rejected by their unchanged
  receipt stamp; the defect is the incompatible upper interval bound for a
  genuine supported acquisition rate, not duplicate handling.
- FusionGraph uses COG yaw for absolute heading updates, initialization/recovery,
  keyframe decisions, docking, and calibration. At 1 Hz those functions silently
  lose this heading source.
- The stationary-heading latch uses ROS time with negative ages clamped fresh,
  so pause/rewind can additionally extend the last stationary result beyond its
  configured lifetime.
- Remediation direction: define `max_dt` from the configured physical acquisition
  rate with tolerance (independent of publication rate), retain a valid prior
  observation across expected intervals, and test 1/5/7/10 Hz plus cached
  republication and time jumps.

## Test coverage gaps

Required regression additions:

- Cached status/fix republication versus new acquisition in the still-open
  behavior, hardware, monitoring, backend, and GUI consumers. Phase B covers
  NavSat projection.
- Fixed -> Float -> no-fix and reverse transitions with adversarial cross-topic
  ordering. Phase B covers both callback orders and exact epoch-local projection;
  the other consumers remain open.
- Explicit SET/OMIT/CLEAR invalidation of accuracy, DOP, heading, baseline, and
  satellite fields through the bridge and every cache.
- ROS time zero, pause, backward/forward jump, negative/future age, and rosbag
  replay for behavior, hardware, and monitoring. Phase A covers FusionGraph's
  future/negative-age/rewind gate but not every replay-mode interaction.
- Unique GNSS queue backlog and late fixed messages after their graph nodes have
  expired. Phase A covers delayed duplicates and the existing historical-node
  timestamp suite remains green.
- COG output at every supported acquisition rate, independently varied ROS
  publication rate, cached duplicate stamps, and stationary-latch clock jumps.
- NTRIP connected without valid RTCM, malformed/zero-cell MSM, caster restart,
  mountpoint/source change, station change, and stale diagnostic removal.
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

1. **Phases A and B complete:** public observation identity, FusionGraph
   receipt deduplication/freshness, and bounded exact NavSat/status association
   are implemented and regression-tested.
2. Make COG interval handling honor all supported acquisition rates and test it
   independently of publication cadence.
3. Restore/replace the UM980 moving-rover policy and make correction-age settings
   effective; define typed correction health and source ownership.
4. Convert behavior, dig trust, LED, monitoring, backend, and GUI consumers to
   bounded provenance-aware state and add reconnect/clock tests.

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

# Read-only update discovery and deployment catalog

Settings → Updates combines installed identities with an explicit **Check now**.
The channel selector changes only which channel is compared. It does not change
the installed channel, source checkout, container references, Watchtower settings
or firmware. There are no install, pull, pin-policy or rollback endpoints here.

## Two levels of evidence

- A **published deployment** comes from the repository's `update-manifests`
  branch. `channels/dev.json` or `channels/stable.json` points to a SHA-256-checked
  document under `manifests/`. The pointer and document are published in one Git
  commit. The document records exact registry index, platform manifest and image
  configuration digests, image revisions, source revision, notes and compatibility.
- Until that catalog exists, the checker compares individual upstream image
  tags. These results are explicitly **bundle unverified**, even when every
  installed image matches. Stable uses the latest published non-prerelease tag,
  never the moving main branch. Missing release images remain unavailable.

The catalog currently covers the first-party ROS2, GUI, GPS and three alternative
LiDAR images on Linux amd64 and arm64. Optional third-party/TF-Luna/MAVROS images
are inventoried but marked not covered during comparison. The declaration covers
the Mowgli hardware backend and required firmware protocol. It does not certify
every physical mower model or provide firmware flashing artifacts.

## Producing and promoting a candidate

`gui/cmd/update-manifest` reads registry manifests and small image configuration
documents; it never downloads image layers. It checks each architecture's source
revision is an ancestor of the candidate and that all relevant build inputs are
unchanged between that image and candidate. Unchanged component images can be
reused. Successful originating image workflows and relevant GUI/ROS2/protocol
checks are required; missing, failed or pending evidence prevents publication.

From a complete repository checkout, as a normal user:

```bash
cd gui
go run ./cmd/update-manifest --repository mowglinext/mowglinext \
  --ref origin/dev --channel dev --version dev --out /tmp/update-candidate
```

`GH_TOKEN` optionally authenticates GitHub metadata reads, and is never sent to
the registry or included in the output. The CLI only writes a candidate directory.

The **Update deployment catalog** workflow tries to assemble candidates after
successful dev/main image or CI jobs and published releases. An incomplete
candidate fails its validation job and retains the existing channel pointer;
another completing workflow retries assembly. Manual dispatch defaults to an
artifact-only dry run. Publishing from dispatch requires an explicit flag and
a dev/main workflow ref. Workflow-run inputs cannot select a fork/PR checkout;
only trusted dev code executes, and only this workflow's own candidate artifact
is promoted. Catalog pushes are serialized and non-forced. A changed dev head
prevents an older candidate from being promoted.

Stable requires the latest published `vX.Y.Z` release, matching source tag and
complete release-tagged images. GUI and sensor pipelines now also run for those
tags. This does not retroactively certify v1.1.0 or publish a stable bundle from
development images.

Compatibility format v1 declares firmware protocol and compose/config schema 1.
Migration and rollback are **manual-review**, not a promise that an older binary
can consume newer state. Installation must remain a later, separately reviewed
host-updater feature with real migration and recovery contracts.

## Checker behavior

`GET /api/system/updates?channel=dev` reads the process-local cached result.
`&check=true` explicitly checks remote metadata, at most once per minute per
channel. A successful check time is retained across failures during the GUI
process lifetime. Restarting the GUI clears this cache. The selected comparison
channel is remembered only in the browser. There is no background registry poll.

Network work has a 45-second request budget; individual HTTP calls have a
15-second timeout and documents have a 4 MiB limit. Image sources are limited to
GHCR and catalog/GitHub hosts are fixed. `UPDATES_REPOSITORY=owner/repo` allows a
host administrator to select a fork catalog; it is not a user-supplied URL.

Only equal immutable identities establish an image match. The checker resolves
older index digests when needed so a changed index with an unchanged host-platform
image is not reported as an update. Equal source SHAs do not hide rebuilt images.
Missing platforms, unknown installed identities, unsupported components and
network failures prevent a blanket up-to-date assertion. A newer dev head or
stable release than the catalog is shown as preparing. Firmware compatibility is
shown separately and uses fresh hardware status.

Tests cover registry platform/config resolution, checksum and schema rejection,
source ancestry and unchanged-image reuse, same-SHA rebuilds, different indexes
with the same platform, absent catalogs, missing platforms, remote failures,
cached read-only routes, and desktop/mobile channel comparisons.

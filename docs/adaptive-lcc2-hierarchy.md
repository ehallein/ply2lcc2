# Adaptive LCC2 hierarchy design

## Partitioning

Generated LCC2 output partitions the original full-detail splats before any
clustering or decimation. Starting with source indices in PLY order, the
partitioner repeatedly chooses the longest centre-bounds axis, stably sorts by
that coordinate and original source index, and splits the count at its midpoint.
The low half is visited before the high half. This gives stable leaf IDs and
ordering, avoids empty children, and guarantees progress. If all centres are
coincident, the source-index tie-break still divides the count deterministically;
the resulting leaves intentionally have overlapping spatial bounds.

Splitting stops when a leaf contains at most `--max-leaf-splats` source splats
(65,536 by default). Generation fails if that invariant is violated.

## Fixed-membership invariant

Each leaf owns one immutable set of original source indices. Its finest node is
that set exactly, and every coarser node is produced by running the existing LOD
algorithm only on splats in the same leaf. No representative can summarize a
splat from another leaf. Atomic parent-to-child replacement is therefore valid:
every node in the chain represents the same fixed source region.

Leaves that cannot be reduced through every requested level keep only their
valid distinct levels. Their chains are fine-aligned with the global ranks, and
generation logs each short chain. Empty or duplicate nodes are not emitted.

## Bounds and error

A leaf bound is computed from its full-detail membership and reused for every
node in its chain. For each Gaussian, it expands the centre by three times the
largest principal standard deviation on all three world axes. This spherical
envelope is conservative under rotation and corresponds to a three-sigma
(99.7% one-dimensional) support assumption. It also keeps each leaf's vertical
extent local rather than copying the scene's full height.

Per-node `lodError` retains the existing accumulated maximum of centre
displacement plus representative-radius difference. Top-level `lodErrors` use
the maximum node error at each rank, so they conservatively summarize leaves.

## Payload layouts

`--lcc2-payload-layout level` is the compatibility default. Leaves are
concatenated in stable ID order at each rank and one SPZ v4 file is written for
the complete rank. Every node range is contiguous.

`--lcc2-payload-layout chunked` groups consecutive complete leaf
representations until adding another would exceed `--max-payload-splats`.
Nodes are never split. A node larger than the configured payload maximum is an
actionable error. Multiple `splatFiles` may therefore belong to the same rank;
each node's `data.3dgs.name`, `start`, and `count` select its exact file and
range. The writer verifies that ranges are contiguous, non-empty, in bounds,
and cover each payload exactly.

## QuestSplat follow-up

QuestSplat's current reader assumes one file per detail rank, so the default
level layout remains compatible. Before enabling chunked packages, QuestSplat
needs a separate reader/streamer change that:

- treats `data.3dgs.name` as the authoritative file index for every node;
- permits several file indices at one `lodLevel`;
- fetches and decompresses payloads independently;
- applies each node's `start` and `count` within the referenced file; and
- caches/releases chunks based on selected leaf chains rather than whole ranks.

No QuestSplat code is changed by ply2lcc2's hierarchy work.

## Garden validation

The supplied `garden.ply` was regenerated in compatibility mode with the
default cluster method, five levels, reduction factor four, and
`--max-leaf-splats 65536`. The source contained 985,239 splats. The former
fixed-grid hierarchy had five cells and its dominant cell contained 41,954,
68,764, 132,770, 318,238, and 964,543 splats from coarse to fine.

The adaptive result has 16 leaves (minimum/median/maximum full-detail membership
61,577 / 61,577 / 61,578):

| Coarse-to-fine rank | Total splats | Nodes | Maximum node splats |
|---:|---:|---:|---:|
| 0 | 50,705 | 16 | 5,334 |
| 1 | 80,986 | 16 | 7,761 |
| 2 | 150,054 | 16 | 12,545 |
| 3 | 340,547 | 16 | 24,678 |
| 4 | 985,239 | 16 | 61,578 |

The compatibility package used five payloads and passed
`splat-transform v3.3.0 meta.lcc2 --stats json null`, which reported the same
counts in finest-to-coarsest order. This validates package structure and SPZ v4
decoding only; visual equivalence was not tested in SuperSplat or QuestSplat.

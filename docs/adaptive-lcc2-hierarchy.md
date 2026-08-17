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
(8,192 by default). `--max-node-diagonal` can request additional extent-driven
splits when a node has at least `--min-split-splats` members. Count limits always
take precedence. Generation fails if the finest-node count invariant is violated.

## Fixed-membership invariant

Finest leaves are grouped bottom-up over the configured ranks. Adjacent leaves
are paired; when a level has an odd count, one three-child parent avoids leaving
a unary branch. A unary promotion is emitted only when a level genuinely has a
single spatial node. Each parent owns the exact union of its children's original
source indices, and its approximation is generated only from that union. Thus
atomic parent-to-visible-children replacement preserves content while allowing
different child regions to refine independently.

The nominal reduction target is retained where possible. If it would make
`sum(child counts) - parent count` exceed `--max-refinement-cost` (20,000 by
default), the parent retains the coarsest available representation that satisfies
the limit. This trades some coarse-rank payload size for bounded runtime jumps.

## Bounds and error

A finest bound is computed from its full-detail membership. For each Gaussian,
it expands the centre by three times the
largest principal standard deviation on all three world axes. This spherical
envelope is conservative under rotation and corresponds to a three-sigma
(99.7% one-dimensional) support assumption. It also keeps each leaf's vertical
extent local. Internal bounds are exact unions of child bounds, so children are
contained by their parents and bounds shrink toward finer descendants.

Per-node `lodError` retains the existing accumulated maximum of centre
displacement plus representative-radius difference, in source coordinate units.
Parent errors are raised to at least the maximum child error, finest errors are
zero, and top-level `lodErrors` use the maximum node error at each rank.

## Payload layouts

`--lcc2-payload-layout level` is the compatibility default. Nodes are
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

Generated hierarchy GUIDs are derived deterministically from node structure and
ranges. The writer also validates exact payload coverage, child containment,
rank transitions, and monotonic errors before committing metadata.

## Garden validation

The 985,239-splat garden fixture was regenerated with the compatibility payload
layout, cluster method, five ranks, `--max-leaf-splats 8192`, and
`--max-refinement-cost 20000`. The former hierarchy had 16 roots, 80 renderable
nodes, 64 unary nodes, identical bounds down each chain, and a dominant-chain
refinement jump as large as 646,305 splats.

The branching result contains 248 renderable nodes: 8 roots, 128 finest spatial
leaves, 120 binary internal nodes, and no unary nodes.

| Coarse-to-fine rank | Nodes | Total splats | Node-count maximum |
|---:|---:|---:|---:|
| 0 | 8 | 48,711 | 8,757 |
| 1 | 16 | 80,986 | 7,761 |
| 2 | 32 | 153,190 | 7,082 |
| 3 | 64 | 343,429 | 6,997 |
| 4 | 128 | 985,239 | 7,698 |

Refinement costs across the 120 internal nodes were 2,502 minimum, 8,924
median, 10,898 p95, and 11,226 maximum. Finest-node bound diagonals were 2.29
minimum, 15.77 median, 51.51 p95, and 87.54 maximum; large values reflect
outlier Gaussian support, while median bounds shrink consistently toward finer
ranks. The package passed writer range/bounds/error validation and
`splat-transform v3.3.0 meta.lcc2 --stats json null`, reporting finest-first
counts `[985239, 343429, 153190, 80986, 48711]` and SH degree 3.

This validates structure and decoding only. Visual equivalence was not tested
in SuperSplat or QuestSplat.

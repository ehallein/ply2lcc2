# Supplied spatial LOD plan

## Existing behaviour

The converter discovers only an unnumbered base PLY followed by `_1.ply`,
`_2.ply`, and so on. Supplied LODs are written as independent whole-scene
root children. The spatial grid built while reading them is not referenced by
that LCC2 metadata. Spatial hierarchy and chunked SPZ output are currently
available only with `--generate-lod`, which creates new decimated or clustered
representations.

## Proposed changes

Recognize a trailing numeric LOD suffix, including names such as `scene_lod0`,
and infer the complete contiguous level set from sibling filenames. For
multi-level LCC2 input, order supplied levels by splat count from coarsest to
finest and build one adaptive kd partition from the finest supplied level.
This is a preservation-first path: supplied records remain unchanged, while a
missing region/rank representation is synthesized locally from the immediate
finer representation. It does not retrain the supplied levels.

## Data flow

1. Discover and validate all supplied PLY levels.
2. Read each independent trained Gaussian set.
3. Build and freeze an adaptive spatial partition from the finest set.
4. Assign every Gaussian centre at every level through the same split planes.
5. Reorder complete Gaussian records by spatial leaf.
6. Fill missing leaf/rank pairs by deterministic moment-matched clustering,
   using the supplied scene-wide adjacent-rank count ratio and at least one
   representative per leaf.
7. Encode bounded groups of complete leaf representations as SPZ v4.
8. Write LCC2 nodes whose file index and start/count reference those chunks.

## Hierarchy strategy

The initial partition is a deterministic longest-axis kd tree. It stops at
`--max-leaf-splats`, or later when the optional extent limit is satisfied.
Every spatial leaf has an independent chain of the supplied representations,
coarsest to finest. Every chain contains a non-empty node at every declared
rank; missing supplied coverage is synthesized only from the same leaf's
immediate finer representation. Bounds use three times the largest Gaussian
principal scale and parents are expanded to include their descendants. Output
is rejected unless every chain is complete and the coarsest root sum equals the
declared coarsest `lodSplats` count.

## SPZ chunking strategy

Within each supplied level, non-empty leaf representations are emitted in
stable leaf order. Adjacent complete representations are grouped until the
next would exceed `--max-payload-splats`; a representation is never split.
Each group is encoded independently as SPZ v4. Metadata ranges are contiguous,
non-empty, in bounds, and cover each payload exactly.

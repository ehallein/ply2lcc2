# ply2lcc2

A high-performance converter for 3D Gaussian Splatting (3DGS) PLY files to LCC and LCC2 packages.

## Features

- **Zero-copy PLY reading**: Memory-mapped file access with SplatView for direct data access
- **Parallel grid building**: OpenMP-parallelized spatial partitioning with thread-local grids
- **Multi-LOD support**: Automatic detection and processing of LOD files (point_cloud_1.ply, point_cloud_2.ply, etc.)
- **Environment support**: Separate processing of environment splats (environment.ply)
- **SH coefficient encoding**: Full support for spherical harmonic coefficients (degree 3)
- **SuperSplat-compatible LCC2 LODs**: Offline hierarchies packaged as validated SPZ v4 payloads

## Build

### CLI Only

```bash
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make -j$(nproc)
```

### LCC2 output

Use `--format lcc2` (or `--lcc2`) to create an LCC2 v0.0.3 package. The
package contains `meta.lcc2`, `LCC2-NOTICE.md`, and its PLY or SPZ payloads
under `data/3dgs/`:

```bash
./ply2lcc2 -i input.ply -o output_lcc2 --format lcc2
```

To store an existing SPZ v4 payload while using its matching PLY for spatial
metadata, add one `--lcc2-payload` per LOD:

```bash
./ply2lcc2 -i garden.ply -o garden_lcc2 --format lcc2 \
  --lcc2-payload garden.spz
```

The converter verifies that each SPZ payload's splat count and SH degree match
the corresponding PLY before writing the package.

### Offline LOD generation

Generated LOD output requires
[`splat-transform`](https://github.com/playcanvas/splat-transform), which
performs the PLY-to-SPZ v4 encoding. Install it first:

```bash
npm install -g @playcanvas/splat-transform
```

Then generate a deterministic hierarchy from a PLY:

```bash
./ply2lcc2 -i garden.ply -o garden_lcc2 --format lcc2 \
  --generate-lod --lod-levels 5 --lod-reduction 4 --lod-method cluster
```

Generated hierarchies partition the original source first. By default, every
independently selectable leaf contains at most 65,536 full-detail splats:

```bash
./ply2lcc2 -i garden.ply -o garden_lcc2 --format lcc2 \
  --generate-lod --max-leaf-splats 65536
```

All representations in a leaf derive exclusively from that leaf's fixed source
membership. The default `level` layout keeps one SPZ per detail rank. The
opt-in `chunked` layout groups complete node representations into bounded files:

```bash
./ply2lcc2 -i garden.ply -o garden_lcc2_chunked --format lcc2 \
  --generate-lod --max-leaf-splats 65536 \
  --lcc2-payload-layout chunked --max-payload-splats 262144
```

QuestSplat currently assumes one payload file per rank. Do not use chunked
packages there until its reader supports each node's `data.3dgs.name` file
index and range independently, including multiple files at one rank.

If `splat-transform` is not on `PATH`, pass its executable explicitly:

```bash
./ply2lcc2 -i garden.ply -o garden_lcc2 --format lcc2 \
  --generate-lod --splat-transform /path/to/splat-transform
```

SPZ input currently requires a matching PLY with the same stem because this
project decodes Gaussian attributes from PLY. All generated SPZ files use the
same SH degree as the source, as required by the PlayCanvas LCC2 reader.
Clustered representatives use zero higher-band SH coefficients; decimated
representatives retain the selected source coefficients. The finest level
preserves the complete source SH data.

`cluster` uses spatial Morton ordering, a colour similarity threshold, weighted
centroids, accumulated alpha, and covariance moment matching. `decimate` uses
spatially stratified importance selection. Generated metadata concatenates
adaptive leaves in stable order at every level and stores `lodError` as the maximum
centre displacement plus representative-radius difference introduced by that
level (accumulated toward coarser levels). Canonical LCC2 metadata is written
finest-first, while the spatial tree nests progressively finer data below each
coarse leaf node. Leaf bounds conservatively include three times the largest
Gaussian principal scale on every world axis and remain identical throughout a
leaf's chain.

The generated directory has this structure:

```text
garden_lcc2/
├── meta.lcc2
├── LCC2-NOTICE.md
└── data/3dgs/
    ├── lod_0.spz
    ├── ...
    └── lod_4.spz
```

Import `meta.lcc2` into SuperSplat rather than opening an individual SPZ file;
the metadata connects the payloads into the spatial LOD hierarchy. You can
also fully decode and validate every LOD with `splat-transform`:

```bash
splat-transform garden_lcc2/meta.lcc2 --stats json null
```

The conversion fails rather than writing a partial package if an SPZ encoder
exits unsuccessfully, omits an output, writes a version other than SPZ v4, or
produces a mismatched splat count or SH degree.

LCC2 output currently includes 3DGS LOD files. An environment PLY is supported
only when the LCC2 splat payloads are also PLY; it cannot be mixed with SPZ
payloads. Collision meshes and trajectory poses remain available only in the
original LCC output.

### With GUI (requires Qt5 or Qt6)

```bash
mkdir build && cd build
cmake .. -DBUILD_GUI=ON
make -j$(nproc)
```

This builds both the CLI (`ply2lcc2`) and GUI (`ply2lcc-gui`) executables.

## Usage

```bash
# Single PLY file (auto-detects point_cloud_1.ply, point_cloud_2.ply, etc. in same dir)
./ply2lcc2 -i /path/to/point_cloud.ply -o /path/to/output_dir

# Custom cell size
./ply2lcc2 -i input.ply -o output --cell-size 50,50

# Single LOD mode (no LOD hierarchy)
./ply2lcc2 -i input.ply -o output --single-lod
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-i <path>` | Input PLY file | Required |
| `-o <path>` | Output LCC directory | Required |
| `-e <path>` | Path to environment.ply | Auto-detect in input dir |
| `-m <path>` | Path to collision.ply | Auto-detect in input dir |
| `--cell-size X,Y` | Grid cell size in meters | 30,30 |
| `--single-lod` | Use only LOD0 even if more exist | false |
| `--format lcc\|lcc2` | Select the output format | `lcc` |
| `--lcc2-payload <path>` | Use a matching PLY or SPZ v4 payload in LCC2 output; repeat per LOD | Input PLY |
| `--generate-lod` | Generate an offline spatial LOD hierarchy (LCC2 only) | false |
| `--splat-transform <path>` | Path to the SPZ encoder used for generated LODs | Search `PATH` |
| `--lod-levels <n>` | Total generated levels, including the original | 5 |
| `--lod-reduction <n>` | Approximate reduction factor per level | 4 |
| `--lod-method cluster\|decimate` | LOD generation strategy | `cluster` |
| `--lod-debug` | Print cluster size/error/rejection details | false |
| `--max-leaf-splats <n>` | Maximum full-detail splats in a generated adaptive leaf | 65536 |
| `--lcc2-payload-layout level\|chunked` | One payload per rank or bounded multi-file ranks | `level` |
| `--max-payload-splats <n>` | Maximum splats per chunked payload; nodes are never split | 262144 |

See [Adaptive LCC2 hierarchy design](docs/adaptive-lcc2-hierarchy.md) for the
partition, bounds, error, and streaming contracts.

## LCC2 attribution

The LCC2 data organization format originated from XGRIDS. This project is an
independent implementation and is not an official XGRIDS implementation. See
[the LCC2 whitepaper](https://github.com/xgrids/LCC2Whitepaper) for the format
specification and its license and redistribution conditions. The LCC2 support
in this project modifies the organization described by the whitepaper by mapping
generated LOD ranges into spatial nodes beneath the root.

## GUI Usage

The GUI provides a user-friendly interface for users unfamiliar with command line tools.

```bash
./ply2lcc-gui
```

### Features

- **File pickers**: Browse for input PLY files and output directory
- **Input filter**: File picker filters for `point_cloud*.ply` files by default
- **Settings panel**:
  - Cell Size X/Y: Grid cell dimensions in meters
  - Single LOD mode: Disable LOD hierarchy
  - Include environment: File picker with path validation (red background if file not found)
  - Include collision: File picker with path validation (red background if file not found)
- **Progress bar**: Real-time conversion progress
- **Log display**: Timestamped conversion messages

When enabling environment or collision, the default path is set to the input directory. The path text box shows a red background if the file doesn't exist.

## Output Files

| File | Description |
|------|-------------|
| `data.bin` | Encoded splat data (32 bytes per splat) |
| `shcoef.bin` | SH coefficients (64 bytes per splat, Quality mode) |
| `index.bin` | Spatial index (cell-to-offset mapping) |
| `meta.lcc` | JSON metadata (bounds, attributes, settings) |
| `attrs.lcp` | Attribute metadata |
| `environment.bin` | Environment splats (if present) |
| `Collision.lci` | Collision mesh with BVH (if present) |

## Architecture

```
ConvertApp (orchestrator)
     │
     ├── SpatialGrid::from_files()
     │   └── PLY Files → SplatBuffer (mmap, zero-copy)
     │       └── Parallel grid building (OpenMP)
     │           - Thread-local grids
     │           - Range computation
     │           - Sequential merge
     │
     ├── GridEncoder::encode()
     │   └── Parallel cell encoding (OpenMP)
     │       - Position, color, scale, rotation
     │       - SH coefficients (11-10-11 bit packing)
     │       → LccData
     │
     └── LccWriter::write()
         └── data.bin, shcoef.bin, index.bin, meta.lcc, attrs.lcp
```

### Key Components

- **SplatBuffer/SplatView**: Zero-copy access to memory-mapped PLY data
- **SpatialGrid**: Grid building with `from_files()` factory, cell indexing, range computation
- **GridEncoder**: Parallel splat encoding, produces `LccData`
- **LccData**: Data container (encoded cells, environment, metadata)
- **LccWriter**: Consolidated file I/O for all LCC output files
- **ConvertApp**: Thin orchestrator for the conversion pipeline

## Performance

- ~2.6x speedup from parallel grid building
- Memory efficient: No intermediate splat storage during grid building
- Scales with available CPU cores via OpenMP

## Testing

```bash
cd build
ctest --output-on-failure
```

## License

MIT

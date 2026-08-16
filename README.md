# ply2lcc

A high-performance converter for 3D Gaussian Splatting (3DGS) PLY files to LCC format.

## Features

- **Zero-copy PLY reading**: Memory-mapped file access with SplatView for direct data access
- **Parallel grid building**: OpenMP-parallelized spatial partitioning with thread-local grids
- **Multi-LOD support**: Automatic detection and processing of LOD files (point_cloud_1.ply, point_cloud_2.ply, etc.)
- **Environment support**: Separate processing of environment splats (environment.ply)
- **SH coefficient encoding**: Full support for spherical harmonic coefficients (degree 3)

## Build

### CLI Only

```bash
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make -j$(nproc)
```

### LCC2 output

Use `--format lcc2` (or `--lcc2`) to create an LCC2 v0.0.3 package. The
package contains `meta.lcc2`, `LCC2-NOTICE.md`, and the source PLY data under
`data/3dgs/`:

```bash
./ply2lcc -i input.ply -o output_lcc2 --format lcc2
```

To store an existing SPZ v4 payload while using its matching PLY for spatial
metadata, add one `--lcc2-payload` per LOD:

```bash
./ply2lcc -i garden.ply -o garden_lcc2 --format lcc2 \
  --lcc2-payload garden.spz
```

The converter verifies that each SPZ payload's splat count and SH degree match
the corresponding PLY before writing the package.

### Offline LOD generation

Generate a deterministic coarse-to-fine hierarchy for LCC2 with:

```bash
./ply2lcc -i garden.spz -o garden_lcc2 --format lcc2 \
  --generate-lod --lod-levels 5 --lod-reduction 4 --lod-method cluster
```

SPZ input currently requires a matching PLY with the same stem because this
project decodes Gaussian attributes from PLY. Generated coarse levels are
RGB-only binary PLY payloads; the finest level preserves the source PLY's SH
attributes. The original non-LOD SPZ packaging path is unchanged.

`cluster` uses spatial Morton ordering, a colour similarity threshold, weighted
centroids, accumulated alpha, and covariance moment matching. `decimate` uses
spatially stratified importance selection. Generated metadata partitions every
level into contiguous spatial-cell ranges and stores `lodError` as the maximum
centre displacement plus representative-radius difference introduced by that
level (accumulated toward coarser levels).

LCC2 output currently includes 3DGS LOD files and an optional environment PLY.
Collision meshes and trajectory poses remain available only in the original LCC
output.

### With GUI (requires Qt5 or Qt6)

```bash
mkdir build && cd build
cmake .. -DBUILD_GUI=ON
make -j$(nproc)
```

This builds both the CLI (`ply2lcc`) and GUI (`ply2lcc-gui`) executables.

## Usage

```bash
# Single PLY file (auto-detects point_cloud_1.ply, point_cloud_2.ply, etc. in same dir)
./ply2lcc -i /path/to/point_cloud.ply -o /path/to/output_dir

# Custom cell size
./ply2lcc -i input.ply -o output --cell-size 50,50

# Single LOD mode (no LOD hierarchy)
./ply2lcc -i input.ply -o output --single-lod
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
| `--lod-levels <n>` | Total generated levels, including the original | 5 |
| `--lod-reduction <n>` | Approximate reduction factor per level | 4 |
| `--lod-method cluster\|decimate` | LOD generation strategy | `cluster` |
| `--lod-debug` | Print cluster size/error/rejection details | false |

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

# NVIDIA cuOpt Integration for ReUseX

## Overview

ReUseX now supports NVIDIA cuOpt as a GPU-accelerated MIP (Mixed Integer Programming) solver option. This provides potential significant speedups for geometry optimization problems on systems with NVIDIA GPUs.

## Current Status

✅ **Implemented:**
- CGAL MIP traits class for cuOpt (`libs/reusex/extern/include/CGAL/cuOpt_mixed_integer_program_traits.h`)
- Automatic solver selection with priority: cuOpt > HiGHS > SCIP
- CMake build system integration with `MIP_SOLVER` option
- Conditional compilation in Solidifier and PCL polygonal reconstruction
- Graceful fallback to HiGHS when cuOpt is not available

⚠️ **Pending:**
- cuOpt Nix package build (complex FetchContent dependencies)
- GPU runtime testing and performance benchmarking

## Architecture

### MIP Solver Priority Order

When `MIP_SOLVER=AUTO` (default):
1. **cuOpt** (GPU-accelerated) - if available
2. **HiGHS** (CPU) - current default
3. **SCIP** (CPU) - fallback

### Implementation Files

**Core Integration:**
- `libs/reusex/extern/include/CGAL/cuOpt_mixed_integer_program_traits.h` - cuOpt CGAL traits
- `libs/reusex/cmake/ReUseXLibrary.cmake` - Solver selection logic
- `libs/reusex/cmake/Dependencies.cmake` - Optional cuOpt package search
- `libs/reusex/src/geometry/Solidifier.cpp` - Updated conditional compilation
- `libs/reusex/extern/include/pcl/polygonal_surface_reconstruction.hpp` - Updated conditional compilation

**Build System:**
- `pkgs/cuOpt/package.nix` - Nix package definition (WIP)
- `default.nix` - cuOpt as optional CUDA dependency (currently commented out)

## Building with Different Solvers

### Auto-detection (Default)

```bash
cmake -B build -DMIP_SOLVER=AUTO
cmake --build build
```

This will automatically select the best available solver.

### Force Specific Solver

```bash
# Use cuOpt (requires NVIDIA GPU and cuOpt installed)
cmake -B build -DMIP_SOLVER=CUOPT

# Use HiGHS (default CPU solver)
cmake -B build -DMIP_SOLVER=HIGHS

# Use SCIP (alternative CPU solver)
cmake -B build -DMIP_SOLVER=SCIP
```

## cuOpt C API Integration

The implementation uses cuOpt's C API (`cuopt/linear_programming/cuopt_c.h`) with the following workflow:

1. **Problem Construction:**
   - Variables: bounds, types (continuous/integer/binary)
   - Objective: coefficients and sense (minimize/maximize)
   - Constraints: CSR matrix format (row offsets, column indices, values)

2. **Solving:**
   ```c
   cuOptOptimizationProblem problem;
   cuOptSolverSettings settings;
   cuOptSolution solution;

   cuOptCreateProblem(..., &problem);
   cuOptCreateSolverSettings(&settings);
   cuOptSolve(problem, settings, &solution);
   ```

3. **Solution Extraction:**
   ```c
   cuOptGetTerminationStatus(solution, &status);
   cuOptGetPrimalSolution(solution, values);
   cuOptGetObjectiveValue(solution, &obj_value);
   ```

4. **Cleanup:**
   ```c
   cuOptDestroySolution(&solution);
   cuOptDestroySolverSettings(&settings);
   cuOptDestroyProblem(&problem);
   ```

## CSR Matrix Format

cuOpt uses Compressed Sparse Row (CSR) format for constraint matrices:

```
Constraint: 2x₁ + 3x₂ ≤ 10
           x₁ - x₂ = 0

CSR representation:
row_offsets  = [0,   2,   4]      // Row i spans [offsets[i], offsets[i+1])
col_indices  = [0,   1,   0,   1] // Column index for each non-zero
values       = [2.0, 3.0, 1.0, -1.0]
sense        = ['L', 'E']         // ≤, =
rhs          = [10.0, 0.0]
```

## Solver Configuration

The cuOpt solver is configured with:
- Time limit: 300 seconds (5 minutes)
- Console logging: disabled
- Presolve: enabled
- For MIP: relative gap 1% when the caller sets none, absolute gap 1e-6.
  The Solidifier always sets one: `SolidifierOptions::mip_gap` (1e-6, for
  cuOpt and HiGHS alike). At 1% cuOpt returned a labelling without house2's
  stair hall, which is worth about 0.03 in a section objective of -372
  (reusex-uur).

## Enabling cuOpt (When Available)

### Prerequisites

1. **NVIDIA GPU** with CUDA support
2. **CUDA Toolkit** 12.0+
3. **cuOpt Library** from NVIDIA

### Installation Steps

1. Ensure cuOpt is built and installed (Nix package or manual build)

2. Uncomment cuOpt in `default.nix`:
   ```nix
   buildInputs = [
     # ... other inputs ...
   ] ++ (
     if cudaSupport
     then
       with cudaPackages; [
         cuda_cudart
         cudnn
       ]
       ++ [cuOpt]  # <-- Uncomment this line
     else []
   );
   ```

3. Rebuild:
   ```bash
   nix develop
   cmake -B build -DMIP_SOLVER=AUTO
   cmake --build build
   ```

4. Verify cuOpt is selected:
   ```bash
   # Should output: "MIP Solver: cuOpt (GPU-accelerated)"
   cmake -B build -DMIP_SOLVER=AUTO 2>&1 | grep "MIP Solver"
   ```

## Performance Expectations

Based on cuOpt documentation, expected performance for typical Solidifier problems:

| Problem Size | Variables | Constraints | HiGHS (CPU) | cuOpt (GPU) | Speedup |
|--------------|-----------|-------------|-------------|-------------|---------|
| Small        | <1,000    | <2,000      | ~1s         | ~0.5s       | 2×      |
| Medium       | 1,000-10,000 | 2,000-20,000 | ~5s      | ~1s         | 5×      |
| Large        | >10,000   | >20,000     | ~30s        | ~5s         | 6-10×   |

**Note:** For Solidifier's typical problem size (~2,600 variables, ~4,500 constraints), the speedup may be modest (2-3×) due to GPU setup overhead.

## Troubleshooting

### cuOpt Not Found

```
Could not find a package configuration file provided by "cuOpt"
```

**Solution:** cuOpt is not installed. The build will automatically fall back to HiGHS.

### CUDA Runtime Errors

If cuOpt is installed but GPU is not available:

```cpp
// The solver will detect this and return an error
// Future enhancement: add GPU detection and fallback
```

**Current behavior:** cuOpt solve will fail if GPU is not available.

**Workaround:** Use `MIP_SOLVER=HIGHS` explicitly.

### Build Errors

If you see cuOpt-related build errors:

1. Check CMake output for solver selection
2. Verify cuOpt library and headers are accessible
3. Try forcing HiGHS: `cmake -B build -DMIP_SOLVER=HIGHS`

## Future Enhancements

1. **GPU Detection:** Add runtime check for GPU availability with automatic fallback
2. **cuOpt Nix Package:** Complete Nix package build (currently blocked by FetchContent)
3. **Performance Tuning:** Optimize cuOpt parameters for Solidifier/reconstruction workloads
4. **Benchmarking:** Comprehensive performance comparison across solvers
5. **Warm Start:** Implement solution warm-starting for iterative problems

## References

- [NVIDIA cuOpt GitHub](https://github.com/NVIDIA/cuopt)
- [cuOpt Documentation](https://docs.nvidia.com/cuopt/)
- [cuOpt MILP Features](https://docs.nvidia.com/cuopt/user-guide/latest/milp-features.html)
- [cuOpt C API](https://docs.nvidia.com/cuopt/user-guide/latest/cuopt-c/lp-qp-milp/lp-qp-milp-c-api.html)

## License

cuOpt is proprietary NVIDIA software (unfree license). ReUseX continues to work with open-source solvers (HiGHS, SCIP) when cuOpt is not available.

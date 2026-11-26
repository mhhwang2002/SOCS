# SOCS
Simulation and Optimization of Complex Systems

A research platform for:
- System modeling
- Discrete-event simulation
- Graph algorithms
- Optimization engines
- Hierarchical decision-making

## RMFS CUDA sweep checks

RMFS uses CPU geometry by default. Set `SOCS_RMFS_CUDA_LIBRARY` to load the
optional CUDA backend for movement admission and extension endpoint clearance.
It checks batches of axis-aligned swept boxes in double precision. The DEVS
event loop, FIFO decisions, and release processing run on the CPU. A configured
library or GPU failure stops the run with an error.

On Windows, run the following in a PowerShell with the Visual Studio x64 C++
build environment and CUDA Toolkit on PATH. Replace `sm_86` with the target
GPU architecture as needed. Run from the repository root with SOCS installed
in the active Python environment.

```powershell
New-Item -ItemType Directory -Force tmp | Out-Null
nvcc -shared -arch=sm_86 -o tmp/rmfs_sweep_cuda.dll examples/RMFS/sweep_cuda.cu
$env:SOCS_RMFS_CUDA_LIBRARY = (Resolve-Path tmp/rmfs_sweep_cuda.dll).Path
python -m examples.RMFS.DEVSModelCreator --robots 10 --orders 10 --rolling-passage allow --crossing-stops allow
python -m pytest tests/examples/RMFS/test_sweep_cuda.py
```

Remove the environment variable with
`Remove-Item Env:SOCS_RMFS_CUDA_LIBRARY` to return to CPU execution. Small
collision batches can take longer on CUDA due to data transfers and kernel
launches.

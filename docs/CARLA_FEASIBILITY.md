# CARLA Feasibility on the Current Machine

**Date:** 2026-09-10 · **Verdict: CARLA CANNOT run on this machine.**
The DDS→CARLA migration proceeds as **refactor + adapter design now**, with the
CARLA runtime deferred to hardware that meets spec (or a cloud GPU box).

---

## 1. Discovered hardware / OS

| Resource | This machine | CARLA 0.9.x requirement | Status |
|---|---|---|---|
| **OS** | Windows 11 Home (build 26200), MSYS2/MinGW shell | Windows 10/11 or Ubuntu 18.04+ | OK (Windows build exists) — the migration brief assumed Linux; it is Windows |
| **CPU** | AMD Ryzen 5 5600H — 6 cores / 12 threads, 3.3 GHz | quad-core min, 8-core recommended | Marginal-OK |
| **RAM** | **11.3 GB total, ~3.8 GB free** | **16 GB min, 32 GB recommended** | **FAIL** — cannot host CARLA server (~4–8 GB) + DDS + tools |
| **GPU** | **AMD Radeon RX 6500M, 4 GB VRAM** (+ 512 MB iGPU) | **NVIDIA, 6 GB VRAM min, 8 GB recommended, CUDA + Vulkan** | **FAIL** — AMD is unsupported by CARLA; 4 GB is below the NVIDIA minimum anyway |
| **GPU driver / Vulkan** | AMD proprietary, Vulkan 1.4.315 present | Vulkan | OK for API, irrelevant given VRAM |
| **Disk (D:)** | 54 GB free / 232 GB | ~20 GB (packaged build + Additional Maps) | Tight but not blocking |
| **Python (DDS venv)** | **3.13.3** | CARLA client wheels: **3.7–3.10 only** | **FAIL for in-venv use** — needs a separate 3.10 interpreter for the CARLA client |
| **Docker** | 29.7.2 installed | `carlasim/carla` image needs Linux + `nvidia-container-toolkit` | Present but **unusable here** (no NVIDIA) |
| **WSL2** | Installed (default v2) | GPU passthrough needs NVIDIA CUDA-on-WSL | Present but **unusable here** |

### Blockers, most severe first
1. **GPU / VRAM.** CARLA renders with Unreal Engine and officially requires an
   **NVIDIA GPU with ≥ 6 GB VRAM** (8 GB recommended) plus CUDA. This machine
   has an **AMD RX 6500M with 4 GB VRAM**. AMD is not a supported CARLA
   platform; even ignoring the vendor, 4 GB is under the minimum. Off-screen /
   low-quality mode does not rescue a 4 GB AMD card for anything but a single
   empty map at a few FPS.
2. **RAM.** 11.3 GB total with < 4 GB free. CARLA server alone wants 4–8 GB;
   running CARLA **and** DDS **and** an editor/tools simultaneously (a stated
   requirement) needs 16–32 GB.
3. **Python version.** The DDS venv is 3.13.3. The `carla` PyPI wheel only
   goes up to Python 3.10 (0.9.15). The CARLA client cannot be `pip install`ed
   into the DDS venv; it would need its own 3.10 venv and a process boundary
   to DDS.

**Conclusion:** installing and running CARLA on this laptop is not viable. Any
attempt would be an unsupported AMD/4 GB/11 GB configuration that thrashes or
fails to launch — not a basis for the "observe → decide → control → feedback"
loop the migration requires.

---

## 2. Recommended CARLA version (for suitable hardware)

**CARLA 0.9.15** (Unreal Engine 4).

- Last stable UE4 release; the most mature and best-documented PythonAPI.
- Python **3.8–3.10** client wheels on PyPI (`pip install carla==0.9.15`).
- `carlasim/carla:0.9.15` Docker image for headless Linux + NVIDIA.
- Ships `agents.navigation.GlobalRoutePlanner`, Traffic Manager, and the
  companion `scenario_runner` — everything the migration's `CarlaRoute` /
  `CarlaScenarioRunner` need.
- Lower VRAM / RAM footprint than the **0.10.x** UE5 line, whose PythonAPI is
  still stabilising and whose GPU demands are higher.

Target host: **NVIDIA RTX 3060 (12 GB) / 4070 or better, 32 GB RAM, 8-core
CPU, 60 GB free disk, Ubuntu 22.04** (native or a cloud GPU instance —
`g5.xlarge` / an RTX-class VM). Windows 11 also works for the non-headless
path.

---

## 3. ROS 2 decision

**ROS 2 is NOT needed at this stage — do not install it.**

- The migration connects a Python client to the CARLA server over CARLA's
  **native RPC** (`carla.Client` / `carla.World`). No message bus is required.
- DDS is a single Python process with an internal decision loop, not a ROS
  node graph. `carla-ros-bridge` would only add value if DDS consumed ROS 2
  topics, needed rosbag capture, or RViz — none of which apply.
- Revisit ROS 2 only if a later phase wants: hardware-in-the-loop, multi-node
  distribution, standard AV tooling (Autoware), or rosbag datasets for
  learned-model training. Recorded here so the decision is explicit.

---

## 4. Docker decision

**Not useful on this machine; useful on the eventual target host.**

- Here: the CARLA Docker image is Linux + NVIDIA-only. With an AMD GPU and
  11 GB RAM, containerisation adds a layer without removing any blocker.
- On a proper NVIDIA/Linux host: `carlasim/carla:0.9.15` + `--gpus all` is the
  **recommended** way to run the server headless and reproducibly, decoupled
  from the DDS client. The `carla_adapter` design below assumes a
  client/server split so this drops in unchanged.

---

## 5. Required software (on the eventual CARLA host)

| Layer | Requirement |
|---|---|
| OS | Ubuntu 22.04 LTS (or Windows 11) |
| GPU stack | NVIDIA driver ≥ 525, CUDA 11.x runtime, Vulkan loader (`libvulkan1`, `vulkan-tools`) |
| System pkgs (Ubuntu) | `libomp5`, `libsdl2-2.0-0`, `xdg-user-dirs`, `fonts-liberation`, `libjpeg8`, `libtiff5` (CARLA 0.9.15 runtime deps); `python3.10`, `python3.10-venv` |
| CARLA client | separate venv: `pip install carla==0.9.15 numpy networkx` (+ `scenario_runner` checkout for scenario authoring) |
| DDS | unchanged — its own 3.13 venv; talks to the CARLA client process via the adapter (in-process if the client venv is 3.10 and hosts DDS too, or over a local socket/shared-memory bridge) |
| Runtime constraints | CARLA server: ~1–2 GB VRAM idle, 3–5 GB populated town; 4–8 GB RAM; 20–60 FPS synchronous at `fixed_delta_seconds=0.05`. Sensor streams (camera/LiDAR) add ~1–2 GB VRAM and meaningful per-tick cost — budget the 10 Hz DDS control loop accordingly. |

---

## 6. Path forward (what this migration does now vs later)

| Now (this machine, no CARLA) | Later (CARLA host) |
|---|---|
| Refactor: extract the DDS **Driver** decision loop out of `PhysicsEngine` behind `interfaces.SensorObservation`; deterministic sim keeps working through the new boundary; full regression suite stays green. | Stand up CARLA 0.9.15 (Docker, headless, NVIDIA). |
| Design + skeleton the `carla_adapter/` package (`WorldBackend`, `ObservationSource`, `ControlSink`, `RouteProvider`, `ScenarioRunner`) with a **DeterministicSim reference backend** proving the boundary, and a guarded `CarlaBackend` skeleton. | Implement `CarlaBackend`: `carla.Client` session, ego + Traffic Manager, `GlobalRoutePlanner` route, actor-state `ObservationSource` first, then real sensor streams. |
| Coordinate-frame + steering-sign + unit conversion **spec and tests** (pure, no CARLA). | Run the CARLA smoke tests; validate observe→decide→control→feedback; measure tick rate + CPU/RAM/GPU/VRAM. |
| Docs + `docs/Progress.md`. | Mark I-migration complete only when the CARLA-backed loop is validated. |

The deterministic simulator is **kept** for CI / unit / regression tests
(ADR-003 invariant V, and the seeded suites). CARLA is the realistic
environment for sensor / traffic / weather experiments and future learned
driving — a different purpose, not a replacement of the test backend.

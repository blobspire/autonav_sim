# Design: `autonav_sim` Showcase Restructure & Autoresearch Knowledge Transfer

- **Date:** 2026-06-17
- **Status:** Draft — awaiting user review
- **Primary repo:** `autonav_sim` (`github.com/blobspire/autonav_sim`, public — owned by the author, to be featured)
- **Secondary repo:** `AutoNav_25-26` (`github.com/KazakhStallion/AutoNav_25-26`, public — the team robot stack)
- **Author/owner:** Cole Wendrowski

## 1. Context & Motivation

The IGVC AutoNav competition ended. The goal is to polish two repos for the next team and for other robotics teams:

1. Turn **`autonav_sim`** from "our robot's simulator that happens to live in its own repo" into a general-purpose **"plug your own robot in"** IGVC simulator — capable, easy to use, and impressive enough to feature on the author's GitHub page.
2. Keep **`AutoNav_25-26`** working against the sim as the **clean, real-world integration example** the next team can copy.
3. **Transfer the autoresearch knowledge** (single-sim and dual-sim) into the sim repo so future teams can use it.

Scope is deliberately limited to **the simulation and the autoresearch**. We do **not** refactor the robot's perception/planning/control code.

## 2. Current State (grounded in audit)

The good news: the hard architectural separation is largely already done.

- `autonav_sim` is **already a standalone public repo on the author's own org** (`blobspire`), separate from the team org (`KazakhStallion`).
- `AutoNav_25-26` **already ejected its embedded sim** (commit *"Remove embedded IGVC competition sim"*) and consumes the standalone sim through a **clean topic-level contract** (`/cmd_vel` in; sensor topics out) plus a `vcs.yaml` that clones both repos into one colcon workspace.
- The coupling is **shallow** — sim ↔ robot communicate over ROS topics, not hardcoded cross-repo file paths.
- The sim is **generic in architecture but team-specific in its defaults**: hardcoded robot name `shogi`, ZED camera topic names + calibration, robot dimensions/dynamics, and absolute local paths in config.
- The **autoresearch subsystem is genuinely strong** (reliability-gated fitness scoring, strict KEEP/DISCARD discipline, two-lane VM-oracle + Jetson-GPU orchestration) but is **embedded inside the ROS package** and **hardwired to the author's machines** (VM names, Jetson IP, `/home/cole.guest` paths, ROS domains in `dual_sim_manifest.json`).
- **Cruft** is present: committed run reports + a 168 KB generated JSON in `docs/`, `__pycache__`, untracked `autoresearch/branches/*` run artifacts.

### Audit findings to confirm-and-fix (Phase 0/1 starting points)

These were surfaced by exploration and should be re-verified at implementation time:

- `igvc_competition_sim/config/dynamics_calibration.yaml` — absolute paths `bag_root: /Users/cole/autonav_bags/...`, `video_root: /Users/cole/Downloads`.
- `igvc_competition_sim/igvc_competition_sim/generate_world.py` — hardcoded model name `shogi`, mesh URI prefix, robot mass/inertia.
- `igvc_competition_sim/igvc_competition_sim/sensor_harness.py` — subscribes to `/model/shogi/odometry`.
- `igvc_competition_sim/igvc_competition_sim/camera_bridge.py` — hardcoded ZED topic names, frame IDs, and intrinsics fallback.
- `igvc_competition_sim/config/igvc_competition_compact.yaml` — robot-specific dimensions (wheel track/radius, sensor offsets, dynamics constants) mixed with course config.
- `.../autoresearch/master/orchestrator.py`, `.../autoresearch/log_experiment.py`, `.../autoresearch/master/dual_sim_manifest.json` — hardcoded VM names, Jetson IP `10.66.0.2`, `/home/cole.guest`, `/home/vtcro`, ROS domains, Docker image/container names.
- `docs/AUTORESEARCH_RUN_REPORT_*.md`, `docs/GAZEBO_*_REPORT.md`, `docs/GAZEBO_WORK_LOG.md`, `docs/gazebo_canon_offline_report.json` — generated artifacts tracked in git.
- `igvc_competition_sim/autoresearch/branches/*` — generated run-branch artifacts (untracked, should stay out).

## 3. Goals & Non-Goals

### Goals
- A newcomer can `git clone autonav_sim`, run one command, and watch a **bundled minimal robot** complete the IGVC course — with **zero external dependencies** and **no GPU** (CPU-only).
- The sim defines a **documented robot-integration contract** ("robot profile") so any team can plug in their own robot.
- `AutoNav_25-26` remains a **working, documented real-world example**, runnable in oracle-perception (CPU) mode and pulled via `vcs.yaml`.
- The **autoresearch** is transferred as a **generic, runnable single-sim framework** plus a **documented, manifest-driven dual-sim reference**.
- The repo is **clean and presentable**: no absolute local paths, no secrets, no committed generated artifacts.
- A **showcase README** communicates capability, quickstart, architecture, and how to use the autoresearch.

### Non-Goals
- Refactoring the robot's perception/planning/control code in `AutoNav_25-26`.
- Making full **CUDA/camera perception** runnable without a Jetson — the Jetson is no longer available; that path is documented, not live-verified.
- Rewriting the sim's core architecture (it is sound).
- Productizing the **dual-sim** into a fully configurable plug-in product — it remains a documented, parameterized reference.
- Adding new course types or CI/CD (noted as possible future work, out of scope here).

## 4. Target Architecture

### 4.1 Repo layout (`autonav_sim`)

```
autonav_sim/
├── README.md                     # showcase: what it is, demo GIF, quickstart, architecture, links
├── docs/
│   ├── quickstart.md             # clone → run the minimal demo in ~5 min
│   ├── robot-integration.md      # THE contract: how to plug your own robot in
│   ├── architecture.md           # bridges, sensor harness, world-gen, scoring/monitor
│   ├── autoresearch.md           # set up + run autoresearch on your robot
│   ├── dual-sim-autoresearch.md  # two-lane VM-oracle + GPU-perception design (reference)
│   └── examples/autonav_25-26.md # real-world integration walkthrough
├── igvc_competition_sim/         # the generic ROS2 sim package (name kept — it IS an IGVC sim)
│   ├── <nodes>                   # world-gen, bridges, harness, monitor, mission — now profile-driven
│   └── profiles/
│       ├── minimal/              # bundled reference-robot profile
│       └── README.md             # what a "robot profile" is
├── examples/
│   └── minimal_robot/            # bundled minimal diff-drive robot pkg (URDF + tiny waypoint nav)
├── autoresearch/                 # lifted OUT of the ROS package; generic framework
│   ├── core/                     # fitness, metrics, experiment-loop, gates (robot-agnostic)
│   ├── single_sim/               # runnable driver (verified in the VM)
│   ├── dual_sim/                 # manifest-driven reference orchestrator + example manifest
│   └── config/                   # example weights / editable-files / gate config
├── launchers/                    # generalized Run_*.command equivalents
├── vcs.yaml                      # pulls example robot stacks (AutoNav_25-26) for the real demo
└── scripts/bootstrap_workspace.sh
```

Two structural moves:
- **(a)** Team-specifics (`shogi`, ZED topic/calibration, dimensions, dynamics, absolute paths) move out of code into a **robot profile**.
- **(b)** `autoresearch/` is **lifted out of the ROS package** into a top-level subsystem, split into generic `core/` vs. robot config.

### 4.2 The robot-profile contract (heart of "plug your own robot in")

A team integrates by adding **one profile directory** plus **their robot's ROS package**. The profile (YAML) declares:

- **Identity & geometry:** robot model name, URDF/mesh source, footprint, wheel track/radius, sensor mount offsets.
- **Sensor topic map:** what the sim publishes *to* the robot — camera (image/depth/info), lidar (scan/cloud), GPS (NavSatFix), odom — each remappable to the robot's expected topic names. (ZED-style names become *one team's choice*, not a sim default.)
- **Command & mission interface:** `/cmd_vel` (Twist) in; mission goals via a generic interface. The minimal robot uses standard Nav2 `navigate_to_pose`; the custom `autonav_interfaces/NavigateToWaypoint` action remains available as an opt-in for the AutoNav profile.
- **Dynamics (optional):** command latency / time constants, defaulting to ideal response when a team does not provide a calibration.

`docs/robot-integration.md` documents this contract. `examples/minimal_robot/` is the worked reference implementation.

### 4.3 Example tiers

1. **Minimal robot** — bundled, CPU-only, zero external deps. `git clone && run` → it drives the IGVC course. The instant show-off demo and the plug-in tutorial. **Live-verified in the VM.**
2. **AutoNav_25-26 (oracle-perception / planning-control)** — pulled via `vcs.yaml`, run with ground-truth perception so it needs no GPU. The *real competition robot* demo we **can still verify** on CPU in the VM.
3. **AutoNav_25-26 (full camera/CUDA perception)** — **documented only** (Jetson unavailable). Honest framing, citing existing run reports as evidence it worked.

### 4.4 Autoresearch

- **Generic `core/`:** extract the reliability gate, fitness function, metrics extraction, experiment loop, and KEEP/DISCARD discipline into a **robot-agnostic, config-driven** framework. A team supplies: editable-files list, fitness weights, course/gate config, and a run command.
- **`single_sim/`:** a runnable driver that executes the loop against a robot profile in the VM. **This tier is live-verified** (at least one full evaluate → score → KEEP/DISCARD cycle).
- **`dual_sim/`:** the two-lane orchestrator (VM oracle-perception lane + GPU perception lane) parameterized off `dual_sim_manifest.json` with **documented placeholders** instead of hardcoded machines/IPs/paths. **Documented reference, not live-verified** (no Jetson). `docs/dual-sim-autoresearch.md` explains the architecture, the DDS routing contract, and the scoring, citing real run reports.

## 5. Key Decisions

| # | Decision | Rationale | Status |
|---|----------|-----------|--------|
| D1 | Bundle a **minimal generic robot** as the demo; reference AutoNav_25-26 as the real example | Instant zero-dep clone-and-run + proof of real capability; keeps sim repo clean | Confirmed by user |
| D2 | Autoresearch = **generic runnable single-sim framework** + **documented dual-sim reference** | Matches reusability goal without over-investing in hardware-specific dual-sim | Confirmed by user |
| D3 | **Single-sim is live-verified** in the existing limactl VMs (CPU); **dual-sim is documented** | Jetson no longer available; honest verification | Confirmed by user |
| D4 | **Skip** the optional CPU dual-sim stand-in | CPU perception without CUDA adds no value here | Confirmed by user |
| D5 | The **AutoNav sim profile lives in `AutoNav_25-26`**, pulled via vcs | Keeps the sim repo clean; honest ownership boundary | **Proposed — confirm in review** |
| D6 | **Keep** the ROS package name `igvc_competition_sim` | It is an IGVC sim; renaming is risky and unnecessary | Proposed |
| D7 | Cleanup via **untrack + `.gitignore`**; full git-history scrub of artifacts is **optional/deferred** | Untracking is safe and reversible; history rewrite of a public repo is disruptive | **Proposed — confirm in review (OQ2)** |

## 6. Verification Strategy

- **"Verified"** means: ran **headless in a limactl VM** (CPU), the mission **completes**, and **scoring output is produced**. The macOS host cannot run Gazebo Fortress directly; all runtime verification happens in the VM.
- **Phase 2:** minimal robot completes the course in the VM.
- **Phase 3:** AutoNav oracle-perception lane completes the course in the VM (regression check that generalization didn't break the real integration).
- **Phase 4:** single-sim autoresearch completes **≥1 full evaluate → score → KEEP/DISCARD cycle** in the VM.
- **Dual-sim:** **not** runtime-verified; correctness is argued via documentation + existing run reports, and the manifest is validated to contain only placeholders (no live credentials/paths).

## 7. Phased Execution Plan

Each phase produces its own implementation plan (via the writing-plans skill) and is independently reviewable. Phase 0 and Phase 1 are the natural first plan.

### Phase 0 — Cleanup & safety *(quick, do first)*
- **Deliverables:** updated `.gitignore`; untrack committed artifacts (run reports, generated JSON, `__pycache__`, `autoresearch/branches/*`); parameterize/remove absolute local paths in config; secrets scan.
- **Acceptance:** no absolute user paths in tracked files; no committed generated artifacts; no secrets; `colcon build` unaffected.

### Phase 1 — Generalize the sim core *(biggest lift)*
- **Deliverables:** robot-profile schema + loader; de-hardcode `shogi`/ZED/dimensions/dynamics into a `profiles/minimal` default; launch args wired to the active profile; first draft of `docs/robot-integration.md`.
- **Acceptance:** sim launches against a named profile with **no code edits** for robot-specifics; the default profile reproduces current behavior; the existing AutoNav run still works (regression).

### Phase 2 — Minimal robot + instant demo
- **Deliverables:** `examples/minimal_robot` package (URDF, diff-drive plant, camera/lidar/gps, minimal Nav2 or simple waypoint follower); `profiles/minimal`; one-command launcher; `docs/quickstart.md`.
- **Acceptance:** fresh clone + bootstrap + **one command** → robot drives the course to completion in the VM, with **no AutoNav dependencies**.

### Phase 3 — AutoNav real-world example
- **Deliverables:** AutoNav sim profile (in `AutoNav_25-26`, per D5); `vcs.yaml` wiring; `docs/examples/autonav_25-26.md`; oracle-perception launcher.
- **Acceptance:** `vcs import` + build + run → AutoNav planning/control completes the course with **oracle perception** in the VM.

### Phase 4 — Autoresearch framework
- **Deliverables:** `autoresearch/core` (generic fitness/metrics/loop/gates); `single_sim` driver; config templates; `dual_sim` manifest-driven reference + `docs/dual-sim-autoresearch.md`.
- **Acceptance:** single-sim autoresearch runs **≥1 full cycle** in the VM against a profile; dual-sim docs complete; manifest contains only placeholders (no hardcoded creds/paths).

### Phase 5 — Docs & showcase polish
- **Deliverables:** showcase `README.md`; `docs/architecture.md`; finalized `robot-integration.md`, `autoresearch.md`, `dual-sim-autoresearch.md`; asset placeholders (demo GIF/screenshots); cross-repo links.
- **Acceptance:** a newcomer can follow the quickstart to the minimal demo; the contract, autoresearch setup, and dual-sim design are all documented; the README reads as a capability showcase.

## 8. Risks & Open Questions

- **R1 — Breaking the working AutoNav integration while generalizing.** Mitigation: run the AutoNav oracle lane in the VM as a regression check at the end of each phase that touches the core.
- **R2 — Minimal robot must be a *credible* demo.** It must exercise the same sim features as a real robot (camera + lidar + gps + mission/scoring), not a toy that bypasses them. Designed in Phase 2 to hit all sensor paths and the course monitor.
- **R3 — Mission-interface generalization.** The sim currently drives missions via the custom `NavigateToWaypoint` action. We must cleanly support standard Nav2 `navigate_to_pose` (minimal robot) *and* the custom action (AutoNav), selected by profile.
- **R4 — Dual-sim cannot be live-verified (no Jetson).** Accepted; documented and evidenced via existing run reports. Docs must state this explicitly.
- **OQ1 (D5):** Confirm the AutoNav sim profile lives in `AutoNav_25-26` (pulled via vcs) vs. in `autonav_sim/profiles/`.
- **OQ2 (D7):** Do we scrub the generated artifacts from git history now, or only untrack going forward?

## 9. Out of Scope
- Robot perception/planning/control refactors.
- New course types / world variety beyond what exists.
- CI/CD, automated test infrastructure, container publishing.
- Full productization of the dual-sim orchestration.

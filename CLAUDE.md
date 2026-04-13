# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

# Required Claude Code Settings (MANDATORY on every machine)

Every machine Ozkan uses must have these values in `~/.claude/settings.json` (Windows: `C:\Users\<user>\.claude\settings.json`). If any are missing, warn the user at session start and offer to add them.

```json
{
  "env": {
    "CLAUDE_CODE_DISABLE_ADAPTIVE_THINKING": "1"
  },
  "model": "opus",
  "effortLevel": "high"
}
```

- **`CLAUDE_CODE_DISABLE_ADAPTIVE_THINKING=1`** — Disables adaptive thinking budget so Claude always uses max reasoning, not dynamically reduced.
- **`model: "opus"`** — Default to Opus across all sessions (highest capability tier).
- **`effortLevel: "high"`** — Maximum effort level for all tasks.

Merge these into existing settings — don't overwrite other keys (`permissions`, `enabledPlugins`, `extraKnownMarketplaces`, etc.).

---

# Obsidian Vault — Second Brain Integration

## Vault Discovery (MANDATORY — run at session start)

Before any work, locate the Obsidian vault. Run this detection in order and use the first match:

```bash
# Try common locations
for candidate in \
  "$HOME/Documents/MyNotes" \
  "G:/My Drive/MyNotes" \
  "$HOME/Google Drive/MyNotes" \
  "$HOME/google-drive/MyNotes" \
  "$HOME/My Drive/MyNotes" \
  "/mnt/google-drive/MyNotes" \
  "$HOME/MyNotes"; do
  if [ -d "$candidate/projects" ] && [ -d "$candidate/claude-code-files" ]; then
    echo "VAULT_FOUND: $candidate"
    break
  fi
done
```

If **no vault is found**, immediately warn the user:
> "I cannot find your Obsidian vault (MyNotes). Please connect your Google Drive or tell me the vault path. Without it, I cannot access your Second Brain, project knowledge, or documentation system."

Do NOT proceed with project onboarding, graphify operations, or vault writes until the vault is accessible.

Store the discovered path as `VAULT_ROOT` for the rest of the session.

## Vault Structure (PARA)

```
[VAULT_ROOT]/
├── inbox/              Quick captures
├── journal/            Monthly journals
├── projects/
│   ├── active/         Current work — graphify output + Index.md
│   ├── ideas/          Not yet started
│   ├── on-hold/        Paused
│   └── completed/      Shipped
├── areas/              Ongoing responsibilities (career, research, website)
├── resources/          Reference material
├── archive/            Cold storage
├── assets/             Images, attachments
├── templates/          Obsidian templates
├── claude-code-files/  Claude Code skills and system docs
│   ├── skills/         Reusable skills
│   ├── SECOND_BRAIN.md Full system documentation
│   └── BASE_CLAUDE.md  This template
└── rookie/             Agent config — DO NOT modify
```

## Project Knowledge Base

Every active project has a folder at `[VAULT_ROOT]/projects/active/[project-name]/` containing:
- **Index.md** — Project summary, god nodes, architecture, quick links
- **GRAPH_REPORT.md** — Graphify structural analysis (god nodes, communities, surprising connections)
- **wiki/** — Auto-generated community articles

Before answering architecture or structural questions about this project, check if `[VAULT_ROOT]/projects/active/aic/GRAPH_REPORT.md` exists and read it for context.

## Documentation Rules

- The vault is the single source of truth for project knowledge and decisions
- Repos hold code-specific CLAUDE.md (build, test, lint, patterns); the vault holds the why and the architecture
- Always use `[[wikilinks]]` in vault markdown files for Obsidian compatibility
- When creating vault files, add YAML frontmatter with at minimum: title, date, tags

---

# Workflow Orchestration

## 1. Plan Mode Default

- Enter plan mode for ANY non-trivial task (3+ steps or architectural decisions)
- If something goes sideways, STOP and re-plan immediately — don't keep pushing
- Use plan mode for verification steps, not just building
- Write detailed specs upfront to reduce ambiguity

## 2. Subagent Strategy

- Use subagents liberally to keep main context window clean
- Offload research, exploration, and parallel analysis to subagents
- For complex problems, throw more compute at it via subagents
- One task per subagent for focused execution

## 3. Self-Improvement Loop

- After ANY correction from the user: update `tasks/lessons.md` with the pattern
- Write rules for yourself that prevent the same mistake
- Ruthlessly iterate on these lessons until mistake rate drops
- Review lessons at session start for relevant project

## 4. Verification Before Done

- Never mark a task complete without proving it works
- Diff behavior between main and your changes when relevant
- Ask yourself: "Would a staff engineer approve this?"
- Run tests, check logs, demonstrate correctness

## 5. Demand Elegance (Balanced)

- For non-trivial changes: pause and ask "is there a more elegant way?"
- If a fix feels hacky: "Knowing everything I know now, implement the elegant solution"
- Skip this for simple, obvious fixes — don't over-engineer
- Challenge your own work before presenting it

## 6. Autonomous Bug Fixing

- When given a bug report: just fix it. Don't ask for hand-holding
- Point at logs, errors, failing tests — then resolve them
- Zero context switching required from the user
- Go fix failing CI tests without being told how

---

# Task Management

1. **Plan First**: Write plan to `tasks/todo.md` with checkable items
2. **Verify Plan**: Check in before starting implementation
3. **Track Progress**: Mark items complete as you go
4. **Explain Changes**: High-level summary at each step
5. **Document Results**: Add review section to `tasks/todo.md`
6. **Capture Lessons**: Update `tasks/lessons.md` after corrections

---

# Core Principles

- **Simplicity First**: Make every change as simple as possible. Impact minimal code.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.

---

# Project: Intrinsic AI for Industry Challenge

## Overview

This is the **official toolkit** for the Intrinsic AI for Industry Challenge — a global robotics competition focused on **dexterous cable manipulation and insertion** for electronics assembly (server trays).

**Our fork:** `git@github.com:ozkannceylan/aic.git`
**Upstream:** `https://github.com/intrinsic-dev/aic`

## Hardware Stack (Simulated)
- **Arm:** Universal Robots UR5e
- **Gripper:** Robotiq Hand-E
- **F/T Sensor:** ATI AXIA80-M20
- **Camera:** 3x wrist-mounted Basler acA2440 (1152x1024 @ 20 FPS)

## Challenge Task
Insert cable connectors (SFP modules and SC plugs) into corresponding ports on a task board. Robot starts with cable already grasped, a few cm from the target.

### Qualification Trials
- **Trial 1 & 2:** SFP module → SFP port (different board poses)
- **Trial 3:** SC plug → SC port (tests generalization)
- Board position, NIC card placement, and component positioning are randomized

### Scoring (per trial, max 100 pts)
| Tier | What | Points |
|------|------|--------|
| Tier 1 | Model validity (loads + responds to action) | 0-1 |
| Tier 2 | Performance: smoothness (0-6), duration (0-12), efficiency (0-6) | up to 24 |
| Tier 2 | Penalties: force >20N (-12), off-limit collision (-24) | negatives |
| Tier 3 | Correct port insertion | 75 |
| Tier 3 | Partial insertion | 38-50 (proportional) |
| Tier 3 | Proximity to port | 0-25 |
| Tier 3 | Wrong port | -12 |

## Repository Structure

```
aic/
├── aic_model/          ← YOUR POLICY GOES HERE
│   └── aic_model/policy.py   ← Implement Policy.insert_cable()
├── aic_example_policies/     ← Reference implementations
│   ├── WaveArm.py            ← Minimal example (just waves)
│   ├── CheatCode.py          ← Uses ground truth TF (dev only)
│   └── RunACT.py             ← LeRobot ACT neural network policy
├── aic_engine/               ← Trial orchestration + scoring
├── aic_controller/           ← Impedance controller (500Hz)
├── aic_adapter/              ← Sensor fusion bridge
├── aic_scoring/              ← Scoring implementation
├── aic_interfaces/           ← ROS2 msg/srv/action definitions
├── aic_description/          ← URDF/SDF robot + environment models
├── aic_gazebo/               ← Gazebo plugins
├── aic_bringup/              ← Launch files
├── aic_assets/               ← 3D models and assets
├── docs/                     ← Challenge documentation
├── docker/                   ← Dockerfiles for submission
└── tasks/                    ← Our task tracking (added by us)
```

## Key Commands

### Run Evaluation Container
```bash
export DBX_CONTAINER_MANAGER=docker
docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest
distrobox create -r --nvidia -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
distrobox enter -r aic_eval
/entrypoint.sh ground_truth:=true start_aic_engine:=true
```

### Run Policy (separate terminal)
```bash
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true \
  -p policy:=aic_example_policies.ros.WaveArm
```

### Tare Force/Torque Sensor (before training)
```bash
ros2 service call /aic_controller/tare_force_torque_sensor std_srvs/srv/Trigger
```

### Rebuild Knowledge Graph (after code changes)
```bash
PYTHONUTF8=1 python -X utf8 -c "from graphify.watch import _rebuild_code; from pathlib import Path; _rebuild_code(Path('.'))"
```

## ROS2 Interface
- **Distribution:** ROS 2 Kilted Kaiju (MUST use this — inter-distro not guaranteed)
- **Action:** `/insert_cable` — called by aic_engine to trigger your policy
- **Topics:** Camera images, joint states, F/T data, TF frames
- **Controller frequency:** 500Hz (accepts policy commands at 10-30Hz)
- **Timeout:** 30 seconds for aic_model node to connect after engine starts

## Strategy

### Primary Approach: Imitation Learning (ACT / LeRobot)
1. Collect demonstrations using teleoperation with `ground_truth:=true`
2. Train ACT model using `lerobot-train`
3. Deploy via RunACT-style policy integration
4. Iterate with domain randomization

### Fallback: Hybrid ACT + RL Fine-tuning
- Use ACT as pre-training, fine-tune with RL (SAC/PPO)
- Sparse reward: successful insertion = +10, else 0
- Domain randomization on board pose, component placement

### Key Insights from Research
- Robot starts near target → this is a **precision insertion** problem, not navigation
- Segmentation masks simplify visual input (vs photorealistic rendering)
- Force/torque feedback is critical — 20N threshold triggers scoring penalty
- Grasp offsets of ~2mm, ~0.04 rad are expected — policy must handle this

## Submission
- Package as Docker container with ROS 2 Lifecycle node named `aic_model`
- Must respond to `/insert_cable` action
- Test locally before submitting
- See `docs/submission.md` for full details

## Important Docs to Read
- `docs/getting_started.md` — Setup guide
- `docs/qualification_phase.md` — What exactly is evaluated
- `docs/scoring.md` — Full scoring breakdown
- `docs/policy.md` — Policy integration guide
- `docs/aic_interfaces.md` — ROS2 interface details
- `docs/task_board_description.md` — Physical specs of connectors/ports

---

## graphify

This project has a graphify knowledge graph at graphify-out/.

Rules:
- Before answering architecture or codebase questions, read graphify-out/GRAPH_REPORT.md for god nodes and community structure
- If graphify-out/wiki/index.md exists, navigate it instead of reading raw files
- After modifying code files in this session, run `PYTHONUTF8=1 python -X utf8 -c "from graphify.watch import _rebuild_code; from pathlib import Path; _rebuild_code(Path('.'))"` to keep the graph current

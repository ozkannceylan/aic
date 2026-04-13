# AI for Industry Challenge — Task Tracking

**Authoritative plan lives in Obsidian vault:** `[VAULT_ROOT]/projects/active/aic/Plan.md`
This file is a lightweight mirror for quick reference during development.

**Registration:** Completed (April 13, 2026)
**Qualification Deadline:** June 30, 2026
**Safe Target:** June 15, 2026

---

## Current Phase: 0 — Environment Setup (April 14-20)

- [ ] Boot Ubuntu, verify NVIDIA drivers + CUDA
- [ ] Install Docker + NVIDIA Container Toolkit + Distrobox + Pixi
- [ ] Clone fork, `pixi install`
- [ ] Pull eval container, run `/entrypoint.sh ground_truth:=true start_aic_engine:=true`
- [ ] Run WaveArm → verify robot waves
- [ ] Run CheatCode → verify successful insertion
- [ ] Run RunACT → see ML policy in action
- [ ] Review scoring output

## Upcoming Phases

| Phase | Dates | Focus |
|-------|-------|-------|
| 1 | April 21-27 | Understand system, run experiments, answer research questions |
| 2 | April 28 - May 11 | Data collection (demos) |
| 3 | May 12-25 | Train first model, first submission |
| 4 | May 26 - June 8 | Iterate and improve |
| 5 | June 9-15 | Final polish |
| 6 | June 16-30 | Buffer |

## Key Technical Notes

- This is a **precision last-cm insertion** problem (robot starts near target)
- **Scoring priority:** Insertion (75pt) >>> Penalties (-36pt) >> Trajectory (24pt)
- Policy runs at 10-30Hz, controller at 500Hz
- 30-second timeout for node connection
- ROS 2 Kilted Kaiju only
- Grasp offsets: ~2mm, ~0.04 rad from nominal

## Resources

- Upstream: https://github.com/intrinsic-dev/aic
- Challenge: https://www.intrinsic.ai/events/ai-for-industry-challenge
- Community: https://discourse.openrobotics.org/c/competitions/ai-for-industry-challenge/129
- LeRobot: https://github.com/huggingface/lerobot

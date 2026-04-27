# Dockerfile.m2 — M2 first-learned-submission image (skeleton).
#
# Two-stage build per ARCHITECTURE.md §8. Stage A installs deps + bakes ACT
# weights. Stage B copies only the pixi runtime closure + source + entrypoint
# + weights, dropping the rattler build cache (~GB).
#
# CHANGES vs Dockerfile.m1:
#   - Two-stage (m1 was single-stage; pixi cache shipped in final image).
#   - Adds LeRobot + Torch to the pixi env in Stage A.        TODO post-T20
#   - Verifies ACTPolicy.from_pretrained at build time.       TODO post-T20
#   - Bakes weights at /ws_aic/weights/approach_act.safetensors.   TODO post-T20
#   - CMD switches policy from CheatCode to TrialOrchestrator.
#
# WHAT IS NOT YET IN THIS FILE (intentional, lands post-T20):
#   1. The actual `pixi add lerobot torch` (or pip-in-pixi) line. Pinning the
#      versions only makes sense once T19 epoch-time measurement chooses a
#      Torch build (CUDA 12.x vs CPU-only for the build env).
#   2. The COPY of the ACT weights file. Weights do not exist until T20
#      finishes training. The COPY line + ACTPolicy.from_pretrained smoke are
#      stubbed below as `# POST-T20` markers.
#   3. INSTALL of any image preprocessing dep (Pillow / cv2) not already
#      pulled by LeRobot. Decide post-T20.
#
# DO build:
#   - Verify Stage A still builds end-to-end (ros + pixi + COPY + pixi install).
#   - Verify Stage B copies cleanly from build alias.
#   - Verify CMD policy parameter syntactically resolves
#     (`aic_model.TrialOrchestrator` → loads module + class — see
#     aic_model/aic_model/aic_model.py:70 `expected_policy_class_name = .split('.')[-1]`).
#
# DO NOT build now: T17 demo collection re-run is using local disk heavily
# (parquet writes). docker build for LeRobot+Torch pulls GBs and contends
# for disk I/O. Build defer-able until after T17 finalize lands.
#
# Tag scheme reminder (B-007 / T-008 / T-016):
#   docker build -f docker/aic_model/Dockerfile.m2 -t \
#     973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/egeliler:submission-m2-$(git -C src/aic rev-parse --short HEAD) .

# ============================================================================
# Stage A: build — pixi env + LeRobot + Torch + (post-T20) weights verify
# ============================================================================
FROM docker.io/library/ros:kilted-ros-core AS build

RUN apt update && apt install -y git && curl -fsSL https://pixi.sh/install.sh | bash
ENV PATH="/root/.pixi/bin:$PATH"

# Source COPYs identical to Dockerfile.m1 — ARCHITECTURE.md §8 keeps the
# package set unchanged. New code (TrialOrchestrator, ApproachPolicy,
# ClassicalInsertion, SafetyWatchdog, ObservationBuilder) is already inside
# aic_model/.
COPY aic_example_policies /ws_aic/src/aic/aic_example_policies
COPY aic_model /ws_aic/src/aic/aic_model
COPY aic_interfaces /ws_aic/src/aic/aic_interfaces
COPY aic_utils /ws_aic/src/aic/aic_utils
COPY pixi.toml pixi.lock /ws_aic/src/aic/
COPY pixi_env_setup.sh /ws_aic/src/aic/

SHELL ["/bin/bash", "-c"]
RUN --mount=type=cache,target=/root/.cache/rattler/cache \
    --mount=type=cache,target=/ws_aic/src/aic/.pixi/build \
    cd /ws_aic/src/aic && pixi install --locked

# POST-T20: install LeRobot + Torch into the pixi env.
# Decide between pixi-add (preferred — locks into pixi.lock) vs pip-in-pixi
# (faster, no lockfile churn) once T19 measurements pick the Torch build.
# Example pip-in-pixi pattern (NOT YET ENABLED):
#   RUN cd /ws_aic/src/aic && pixi run pip install \
#         lerobot==0.5.1 \
#         torch==2.4.* --index-url https://download.pytorch.org/whl/cu121

# POST-T20: bake the ACT weights from T20 cloud training.
# Source path on host: src/aic/aic_model/weights/approach_act.safetensors
# Destination (referenced by ApproachPolicy.__init__ once it lands): /ws_aic/weights/approach_act.safetensors
#   COPY aic_model/weights/approach_act.safetensors /ws_aic/weights/approach_act.safetensors

# POST-T20: build-time smoke that the weights load.
# Catches torchcodec / FFmpeg dep drift early (T-021) and confirms the
# checkpoint format matches LeRobot's expected schema.
#   RUN cd /ws_aic/src/aic && pixi run python -c "\
#     from lerobot.common.policies.act.modeling_act import ACTPolicy; \
#     ACTPolicy.from_pretrained('/ws_aic/weights/approach_act.safetensors'); \
#     print('OK: ACT weights load + import chain healthy')"

# ============================================================================
# Stage B: runtime — pixi closure + source + entrypoint + (post-T20) weights
# ============================================================================
FROM docker.io/library/ros:kilted-ros-core AS runtime

# Copy only the resolved pixi env, not the rattler cache or build artifacts.
COPY --from=build /ws_aic/src/aic/.pixi/envs/default /ws_aic/src/aic/.pixi/envs/default

# Source packages — identical layout to build stage so module resolution works
# the same way it did in M1.
COPY --from=build /ws_aic/src/aic/aic_example_policies /ws_aic/src/aic/aic_example_policies
COPY --from=build /ws_aic/src/aic/aic_model /ws_aic/src/aic/aic_model
COPY --from=build /ws_aic/src/aic/aic_interfaces /ws_aic/src/aic/aic_interfaces
COPY --from=build /ws_aic/src/aic/aic_utils /ws_aic/src/aic/aic_utils
COPY --from=build /ws_aic/src/aic/pixi.toml /ws_aic/src/aic/pixi.toml
COPY --from=build /ws_aic/src/aic/pixi.lock /ws_aic/src/aic/pixi.lock
COPY --from=build /ws_aic/src/aic/pixi_env_setup.sh /ws_aic/src/aic/pixi_env_setup.sh

# POST-T20: copy the baked weights from build stage.
#   COPY --from=build /ws_aic/weights /ws_aic/weights

# Pixi binary itself — runtime stage uses `pixi run --as-is` so the binary
# must exist in PATH. Smaller alternative: drop pixi and exec the env's
# python directly. Decision deferred (M2 keeps the M1 entrypoint verbatim
# to avoid retesting Zenoh auth).
COPY --from=build /root/.pixi /root/.pixi
ENV PATH="/root/.pixi/bin:$PATH"

WORKDIR /ws_aic/src/aic

# Entrypoint heredoc copied verbatim from Dockerfile.m1. Same Zenoh auth
# path, same RMW pin, same env-var contract (AIC_ROUTER_ADDR / AIC_MODEL_PASSWD /
# AIC_ENABLE_ACL). Do NOT modify without re-testing the auth flow on portal —
# M1 burned an evening on this in Phase 0.
COPY --chmod=755 <<"EOF" /entrypoint.sh
#!/bin/bash
set -e

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

if [[ -z "$AIC_ROUTER_ADDR" ]]; then
  echo "AIC_ROUTER_ADDR must be provided"
  exit 1
fi

should_enable_acl() {
  [[ "$AIC_ENABLE_ACL" == "true" || "$AIC_ENABLE_ACL" == "1" ]]
}

# prepare the credentials file
if should_enable_acl; then
  if [[ ! (-n "$AIC_MODEL_PASSWD" ) ]]; then
    echo "AIC_MODEL_PASSWD must be provided"
    exit 1
  fi
  echo "model:$AIC_MODEL_PASSWD" >> /credentials.txt
fi

# start the model
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/'"$AIC_ROUTER_ADDR"'"]'
ZENOH_CONFIG_OVERRIDE+=';transport/shared_memory/enabled=false'
if should_enable_acl; then
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="model"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="'"$AIC_MODEL_PASSWD"'"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="/credentials.txt"'
fi
export ZENOH_CONFIG_OVERRIDE
echo "ZENOH_CONFIG_OVERRIDE=$ZENOH_CONFIG_OVERRIDE"
exec pixi run --as-is ros2 run aic_model aic_model "$@"
EOF

ENTRYPOINT ["/entrypoint.sh"]

# Default policy switches from CheatCode (M1 placeholder) to TrialOrchestrator
# (M2 hybrid). Module path: aic_model.TrialOrchestrator → loader takes the
# last `.`-segment as the class name (aic_model.py:70). The alias file at
# aic_model/aic_model/TrialOrchestrator.py re-exports the class from
# orchestrator.py (T22 commit b92a517).
CMD ["--ros-args", "-p", "policy:=aic_model.TrialOrchestrator", "-p", "use_sim_time:=true"]

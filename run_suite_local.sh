#!/usr/bin/env bash
set -euo pipefail

TASK_NAME="seat-floating-microtube-foam-rack-in-benchtop-ultrasonic-water-bath-and-toggle-degas"
TASK_FILE="seat_floating_microtube_foam_rack_in_benchtop_ultrasonic_water_bath_and_toggle_degas.py"
REPO="/home/freax/Documents/GitHub/bt-ultrasonic-water-bath"
EVIDENCE="/mnt/hdd/non-dream-80-evidence/final/${TASK_NAME}"
LOCAL_RUNS="$REPO/runs"
EVIDENCE_RUNS="$EVIDENCE/runs"
mkdir -p "$LOCAL_RUNS" "$EVIDENCE_RUNS"

cd "$REPO"

run_one() {
  local name="$1"
  local policy="$2"
  local steps="$3"
  local store_flag="$4"
  local seed="${5:-0}"
  local extra_env="${6:-}"

  echo "=== [Local RTX 3090] Submitting run: $name (policy=$policy, steps=$steps, seed=$seed, env=$extra_env) ==="

  rm -rf "$LOCAL_RUNS/$name" "$LOCAL_RUNS/$name".*
  rm -rf "$EVIDENCE_RUNS/$name" "$EVIDENCE_RUNS/$name".*

  local extra_args=()
  if [ "$store_flag" = "frames" ]; then
    extra_args+=(--store-frames)
  fi

  find "tasks/${TASK_NAME}" -maxdepth 1 -type f -exec sha256sum {} + > "$LOCAL_RUNS/${name}.pre-run.sha256"
  date -u +%FT%TZ > "$LOCAL_RUNS/${name}.started"

  local env_cmd="env PYTHONDONTWRITEBYTECODE=1"
  if [ -n "$extra_env" ]; then
    env_cmd="$env_cmd $extra_env"
  fi

  $env_cmd /home/freax/agentwork/ctl-isaac/gpu-run \
    --name "${TASK_NAME}_${name}" \
    --ram-gib 9.0 \
    --vram-gib 6.0 \
    --queue-timeout 86400 \
    --timeout 7200 \
    -- /home/freax/IsaacLab-Arena/.venv/bin/inspect-robots run \
    --embodiment yam_arena \
    --policy "$policy" \
    -E task="tasks/${TASK_NAME}/${TASK_FILE}" \
    -E object_jitter=0 \
    -E control_hz=20 \
    -E cam_width=640 \
    -E cam_height=352 \
    --instruction "Transfer the floating microtube foam rack into the ultrasonic bath tank, close the hinged acrylic lid flush, and depress DEGAS." \
    --scorer task_score \
    --max-steps "$steps" \
    --seed "$seed" \
    --no-prompt \
    "${extra_args[@]}" \
    --log-dir "$LOCAL_RUNS/$name" > "$LOCAL_RUNS/${name}.launcher.log" 2>&1 || true

  date -u +%FT%TZ > "$LOCAL_RUNS/${name}.finished"
  find "tasks/${TASK_NAME}" -maxdepth 1 -type f -exec sha256sum {} + > "$LOCAL_RUNS/${name}.post-run.sha256"

  cp -r "$LOCAL_RUNS/$name"* "$EVIDENCE_RUNS/" || true
  echo "=== [Local RTX 3090] Run $name finished! ==="
}

case "${1:-all}" in
  all)
    echo "Running full suite locally through gpu-pool..."
    run_one "demo_1" "yam_scripted" 450 "frames" 0 ""
    run_one "demo_2" "yam_scripted" 450 "frames" 1 ""
    run_one "neg_premature_degas" "yam_scripted" 450 "" 0 "DEMO_VARIANT=premature_degas"
    run_one "neg_no_release" "yam_scripted" 450 "" 0 "DEMO_VARIANT=no_release"
    run_one "baseline_dream1" "dream" 450 "" 0 ""
    ;;
  demo_1)
    run_one "demo_1" "yam_scripted" 450 "frames" 0 ""
    ;;
  demo_2)
    run_one "demo_2" "yam_scripted" 450 "frames" 1 ""
    ;;
  neg_premature_degas)
    run_one "neg_premature_degas" "yam_scripted" 450 "" 0 "DEMO_VARIANT=premature_degas"
    ;;
  neg_no_release)
    run_one "neg_no_release" "yam_scripted" 450 "" 0 "DEMO_VARIANT=no_release"
    ;;
  baseline_dream1)
    run_one "baseline_dream1" "dream" 450 "" 0 ""
    ;;
  *)
    echo "Unknown target: $1"
    exit 1
    ;;
esac

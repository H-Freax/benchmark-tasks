#!/usr/bin/env bash
source /home/freax/IsaacLab-Arena/yam.env
cd /home/freax/Documents/GitHub/bt-pcr-magnetic-separation-stand
EVID="/mnt/hdd/non-dream-80-evidence/final/position-and-lock-96-well-pcr-cooler-rack-on-magnetic-separation-stand"

# 1. demo_2 (seed 1)
mkdir -p "$EVID/demo_2"
rm -rf "$EVID/demo_2"/*
/home/freax/agentwork/ctl-isaac/gpu-run \
  --name pcr_mag_demo_2 \
  --ram-gib 9.0 \
  --vram-gib 6.0 \
  --timeout 7200 \
  --queue-timeout 86400 \
  -- \
  inspect-robots run --embodiment yam_arena \
  -E task=tasks/position-and-lock-96-well-pcr-cooler-rack-on-magnetic-separation-stand/position_and_lock_96_well_pcr_cooler_rack_on_magnetic_separation_stand.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Transfer the 96-well PCR cooler rack onto the magnetic separation stand and engage the slide lock bar." \
  --policy yam_scripted --scorer task_score \
  --log-dir "$EVID/demo_2" \
  --seed 1 \
  --no-prompt \
  --max-steps 500

# 2. no_seat (negative control variant)
mkdir -p "$EVID/no_seat"
rm -rf "$EVID/no_seat"/*
/home/freax/agentwork/ctl-isaac/gpu-run \
  --name pcr_mag_no_seat \
  --ram-gib 9.0 \
  --vram-gib 6.0 \
  --timeout 7200 \
  --queue-timeout 86400 \
  -- \
  env DEMO_VARIANT=no_seat \
  inspect-robots run --embodiment yam_arena \
  -E task=tasks/position-and-lock-96-well-pcr-cooler-rack-on-magnetic-separation-stand/position_and_lock_96_well_pcr_cooler_rack_on_magnetic_separation_stand.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Transfer the 96-well PCR cooler rack onto the magnetic separation stand and engage the slide lock bar." \
  --policy yam_scripted --scorer task_score \
  --log-dir "$EVID/no_seat" \
  --seed 0 \
  --no-prompt \
  --max-steps 500

# 3. dream_1 (baseline policy control)
mkdir -p "$EVID/dream_1"
rm -rf "$EVID/dream_1"/*
/home/freax/agentwork/ctl-isaac/gpu-run \
  --name pcr_mag_dream_1 \
  --ram-gib 9.0 \
  --vram-gib 6.0 \
  --timeout 7200 \
  --queue-timeout 86400 \
  -- \
  inspect-robots run --embodiment yam_arena \
  -E task=tasks/position-and-lock-96-well-pcr-cooler-rack-on-magnetic-separation-stand/position_and_lock_96_well_pcr_cooler_rack_on_magnetic_separation_stand.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Transfer the 96-well PCR cooler rack onto the magnetic separation stand and engage the slide lock bar." \
  --policy dream --scorer task_score \
  --log-dir "$EVID/dream_1" \
  --seed 0 \
  --no-prompt \
  --max-steps 300

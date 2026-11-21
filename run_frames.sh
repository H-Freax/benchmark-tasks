#!/usr/bin/env bash
source /home/freax/IsaacLab-Arena/yam.env
cd /home/freax/Documents/GitHub/bt-pcr-magnetic-separation-stand
mkdir -p /mnt/hdd/non-dream-80-evidence/final/position-and-lock-96-well-pcr-cooler-rack-on-magnetic-separation-stand/demo_1
rm -rf /mnt/hdd/non-dream-80-evidence/final/position-and-lock-96-well-pcr-cooler-rack-on-magnetic-separation-stand/demo_1/*

/home/freax/agentwork/ctl-isaac/gpu-run \
  --name pcr_mag_stand_demo_1 \
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
  --log-dir /mnt/hdd/non-dream-80-evidence/final/position-and-lock-96-well-pcr-cooler-rack-on-magnetic-separation-stand/demo_1 \
  --store-frames \
  --seed 0 \
  --no-prompt \
  --max-steps 500

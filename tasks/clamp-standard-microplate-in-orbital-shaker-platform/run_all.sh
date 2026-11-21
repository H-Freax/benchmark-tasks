#!/usr/bin/env bash
# Full qualification runner for clamp-standard-microplate-in-orbital-shaker-platform
set -e

GPU_RUN="/home/freax/agentwork/ctl-isaac/gpu-run --queue-timeout 86400 --ram-gib 9.0 --vram-gib 6.0"
INSPECT="/home/freax/IsaacLab-Arena/.venv/bin/inspect-robots"
T="/home/freax/Documents/GitHub/bt-clamp-microplate/tasks/clamp-standard-microplate-in-orbital-shaker-platform"
E="/mnt/hdd/non-dream-80-evidence/final/clamp-standard-microplate-in-orbital-shaker-platform"

mkdir -p "$E"

echo "=== RUNNING DEMO 1 (Seed 0, 320x176) ==="
rm -rf "$E/demo1"
$GPU_RUN env PYTHONPATH=$T $INSPECT run \
  --embodiment yam_arena \
  --policy yam_scripted \
  -E task="$T/clamp_standard_microplate_in_orbital_shaker_platform.py" \
  -E object_jitter=0 \
  -E control_hz=20 \
  -E cam_width=320 \
  -E cam_height=176 \
  --seed 0 \
  --instruction "Seat the microplate in the orbital shaker nest, lock the retaining clamp, and turn on the shaker." \
  --scorer task_score \
  --max-steps 300 \
  --store-frames --no-prompt \
  --log-dir "$E/demo1" > "$E/demo1.log" 2>&1
echo "demo1 completed: $(grep 'task_score:' "$E/demo1.log" | tail -n 1)"

echo "=== RUNNING DEMO 2 (Seed 1, 320x176) ==="
rm -rf "$E/demo2"
$GPU_RUN env PYTHONPATH=$T $INSPECT run \
  --embodiment yam_arena \
  --policy yam_scripted \
  -E task="$T/clamp_standard_microplate_in_orbital_shaker_platform.py" \
  -E object_jitter=0 \
  -E control_hz=20 \
  -E cam_width=320 \
  -E cam_height=176 \
  --seed 1 \
  --instruction "Seat the microplate in the orbital shaker nest, lock the retaining clamp, and turn on the shaker." \
  --scorer task_score \
  --max-steps 300 \
  --store-frames --no-prompt \
  --log-dir "$E/demo2" > "$E/demo2.log" 2>&1
echo "demo2 completed: $(grep 'task_score:' "$E/demo2.log" | tail -n 1)"

echo "=== RUNNING CTRL NO SEAT ==="
rm -rf "$E/ctrl_no_seat"
$GPU_RUN env PYTHONPATH=$T DEMO_VARIANT=no_seat $INSPECT run \
  --embodiment yam_arena \
  --policy yam_scripted \
  -E task="$T/clamp_standard_microplate_in_orbital_shaker_platform.py" \
  -E object_jitter=0 \
  -E control_hz=20 \
  -E cam_width=320 \
  -E cam_height=176 \
  --instruction "Seat the microplate in the orbital shaker nest, lock the retaining clamp, and turn on the shaker." \
  --scorer task_score \
  --max-steps 300 \
  --store-frames --no-prompt \
  --log-dir "$E/ctrl_no_seat" > "$E/ctrl_no_seat.log" 2>&1
echo "ctrl_no_seat completed: $(grep 'task_score:' "$E/ctrl_no_seat.log" | tail -n 1)"

echo "=== RUNNING CTRL NO CLAMP ==="
rm -rf "$E/ctrl_no_clamp"
$GPU_RUN env PYTHONPATH=$T DEMO_VARIANT=no_clamp $INSPECT run \
  --embodiment yam_arena \
  --policy yam_scripted \
  -E task="$T/clamp_standard_microplate_in_orbital_shaker_platform.py" \
  -E object_jitter=0 \
  -E control_hz=20 \
  -E cam_width=320 \
  -E cam_height=176 \
  --instruction "Seat the microplate in the orbital shaker nest, lock the retaining clamp, and turn on the shaker." \
  --scorer task_score \
  --max-steps 300 \
  --store-frames --no-prompt \
  --log-dir "$E/ctrl_no_clamp" > "$E/ctrl_no_clamp.log" 2>&1
echo "ctrl_no_clamp completed: $(grep 'task_score:' "$E/ctrl_no_clamp.log" | tail -n 1)"

echo "=== RUNNING CTRL NO RELEASE ==="
rm -rf "$E/ctrl_no_release"
$GPU_RUN env PYTHONPATH=$T DEMO_VARIANT=no_release $INSPECT run \
  --embodiment yam_arena \
  --policy yam_scripted \
  -E task="$T/clamp_standard_microplate_in_orbital_shaker_platform.py" \
  -E object_jitter=0 \
  -E control_hz=20 \
  -E cam_width=320 \
  -E cam_height=176 \
  --instruction "Seat the microplate in the orbital shaker nest, lock the retaining clamp, and turn on the shaker." \
  --scorer task_score \
  --max-steps 300 \
  --store-frames --no-prompt \
  --log-dir "$E/ctrl_no_release" > "$E/ctrl_no_release.log" 2>&1
echo "ctrl_no_release completed: $(grep 'task_score:' "$E/ctrl_no_release.log" | tail -n 1)"

echo "=== EXTRACTING FRAMES & ENCODING VIDEO ==="
python3 -c "
import numpy as np, glob, subprocess
from PIL import Image
f_dirs = glob.glob('/demo1/frames/*')
if f_dirs:
    top_files = sorted(glob.glob(f'{f_dirs[-1]}/*top_cam*.npy'))
    if top_files:
        Image.fromarray(np.load(top_files[0])).save('/setup.png')
        Image.fromarray(np.load(top_files[-1])).save('/final_frame.png')
        subprocess.run('cp -f /*.png /', shell=True)
        cmd = [
            'ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-s', '320x176', '-pix_fmt', 'rgb24', '-r', '20',
            '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', '/demo.mp4'
        ]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        for f in top_files:
            proc.stdin.write(np.load(f).tobytes())
        proc.stdin.close()
        proc.wait()
        subprocess.run('cp -f /demo.mp4 /demo.mp4', shell=True)
"

=== RUNNING DREAM-1 POLICY ==="
rm -rf "$E/dream1"
$GPU_RUN env PYTHONPATH=$T $INSPECT run \
  --embodiment yam_arena \
  --policy dream \
  -E task="$T/clamp_standard_microplate_in_orbital_shaker_platform.py" \
  -E object_jitter=0 \
  -E control_hz=20 \
  -E cam_width=320 \
  -E cam_height=176 \
  --seed 0 \
  --instruction "Seat the microplate in the orbital shaker nest, lock the retaining clamp, and turn on the shaker." \
  --scorer task_score \
  --max-steps 300 \
  --store-frames --no-prompt \
  --log-dir "$E/dream1" > "$E/dream1.log" 2>&1 || true
echo "dream1 completed: $(grep 'task_score:' "$E/dream1.log" | tail -n 1)"

echo "=== ALL QUALIFICATION RUNS COMPLETE ==="

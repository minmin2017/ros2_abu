#!/usr/bin/env bash
# Evaluate the GPU rendering path used by Gazebo + RViz on this WSLg setup.
# Assumes a sim is already running (e.g. `sim`, `simfull`, or `simnav`).
#
# Pass/fail checklist:
#   - direct rendering = Yes, renderer mentions D3D12 + NVIDIA
#   - LIBGL_ALWAYS_SOFTWARE=0, OGRE_RTT_MODE=FBO
#   - gzserver shows up in nvidia-smi processes or GPU memory is non-trivial
#   - /camera/image_raw publishes > 15 Hz
#   - /scan publishes ~10 Hz with finite (non -inf) ranges  (lidar must stay CPU)

set -u

hr() { printf '\n── %s ──\n' "$1"; }

hr "Display / OpenGL"
glxinfo -B 2>/dev/null | grep -E "direct rendering|renderer string|Accelerated|Video memory" || echo "glxinfo missing"

hr "Relevant env vars"
env | grep -E '^(LIBGL_ALWAYS_SOFTWARE|GALLIUM_DRIVER|OGRE_RTT_MODE|MESA_D3D12_DEFAULT_ADAPTER_NAME|MESA_GL_VERSION_OVERRIDE|DISPLAY|WAYLAND_DISPLAY)=' | sort

hr "GPU usage"
nvidia-smi --query-gpu=name,utilization.gpu,memory.used --format=csv,noheader 2>/dev/null || echo "nvidia-smi missing"

hr "Gazebo realtime factor (2s sample)"
timeout 2 gz stats -p 2>/dev/null | tail -3 || echo "gz stats unavailable (sim not running?)"

hr "/camera/image_raw publish rate (6s sample)"
timeout 6 ros2 topic hz /camera/image_raw 2>&1 | tail -6

hr "/scan publish rate + sanity check (6s sample)"
timeout 6 ros2 topic hz /scan 2>&1 | tail -4
echo "(one /scan range sample — should be finite, NOT -inf on WSLg:)"
timeout 3 ros2 topic echo --once --field ranges /scan 2>&1 | head -c 200
echo

hr "Done"

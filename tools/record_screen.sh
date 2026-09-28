#!/bin/bash
# Record an X11 window (default: the Unity sim) to an H.264 mp4.
#
# Usage: tools/record_screen.sh <out.mp4> <duration_s> [window_name_regex]
#   window_name_regex  matched against top-level window names (default: Unity build names)
# DISPLAY defaults to :20 (Chrome Remote Desktop). Stop early with Ctrl-C; the file stays valid.
set -euo pipefail

if [ $# -lt 2 ]; then
  sed -n '4,6p' "$0"
  exit 1
fi

output="$1"
duration="$2"
window_pattern="${3:-pros_twin|Proly|Unity}"
export DISPLAY="${DISPLAY:-:20}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

# Apps register several windows under one name (leaders, 10x10 helpers), so pick the
# largest *viewable* match.
window_id=""
largest_area=0
for candidate in $(xwininfo -root -tree | grep -E "\"[^\"]*(${window_pattern})[^\"]*\"" | awk '{ print $1 }'); do
  info="$(xwininfo -id "$candidate" 2>/dev/null)" || continue
  grep -q 'Map State: IsViewable' <<<"$info" || continue
  area=$(( $(awk '/Width:/ { print $NF }' <<<"$info") * $(awk '/Height:/ { print $NF }' <<<"$info") ))
  if [ "$area" -gt "$largest_area" ]; then
    largest_area="$area"
    window_id="$candidate"
  fi
done
if [ -z "$window_id" ]; then
  echo "No viewable window matching '${window_pattern}' on ${DISPLAY}" >&2
  exit 1
fi

geometry="$(xwininfo -id "$window_id")"
left="$(awk '/Absolute upper-left X/ { print $NF }' <<<"$geometry")"
top="$(awk '/Absolute upper-left Y/ { print $NF }' <<<"$geometry")"
width="$(awk '/Width:/ { print $NF }' <<<"$geometry")"
height="$(awk '/Height:/ { print $NF }' <<<"$geometry")"
# libx264 with yuv420p needs even dimensions.
width=$(( width / 2 * 2 ))
height=$(( height / 2 * 2 ))

echo "Recording window ${window_id} (${width}x${height}+${left}+${top}) on ${DISPLAY} for ${duration}s -> ${output}"
ffmpeg -hide_banner -loglevel error -y -f x11grab -framerate 30 \
  -video_size "${width}x${height}" -i "${DISPLAY}+${left},${top}" -t "$duration" \
  -c:v libx264 -crf 23 -preset veryfast -pix_fmt yuv420p "$output"

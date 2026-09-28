#!/bin/bash
# Convert an mp4 into a small, good-looking GIF for a README (two-pass palette).
#
# Usage: tools/make_gif.sh <in.mp4> <out.gif> [start_s] [duration_s] [width_px] [speed] [fps]
#   start_s     seek offset in seconds             (default 0)
#   duration_s  clip length in *source* seconds    (default: whole video)
#   width_px    output width, height keeps aspect  (default 560)
#   speed       playback speed-up factor           (default 1)
#   fps         output frame rate                  (default 12)
set -euo pipefail

if [ $# -lt 2 ]; then
  sed -n '4,9p' "$0"
  exit 1
fi

input="$1"
output="$2"
start="${3:-0}"
duration="${4:-}"
width="${5:-560}"
speed="${6:-1}"
fps="${7:-12}"

trim_args=(-ss "$start")
if [ -n "$duration" ]; then
  trim_args+=(-t "$duration")
fi

filters="setpts=PTS/${speed},fps=${fps},scale=${width}:-2:flags=lanczos"
palette="$(mktemp --suffix=.png)"
trap 'rm -f "$palette"' EXIT

ffmpeg -hide_banner -loglevel error -y "${trim_args[@]}" -i "$input" \
  -vf "${filters},palettegen=stats_mode=diff" "$palette"
ffmpeg -hide_banner -loglevel error -y "${trim_args[@]}" -i "$input" -i "$palette" \
  -lavfi "${filters}[frames];[frames][1:v]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" \
  "$output"

echo "$output: $(du -h "$output" | cut -f1)"

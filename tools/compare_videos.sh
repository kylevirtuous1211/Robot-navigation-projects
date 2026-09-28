#!/bin/bash
# Put several runs side by side in one labeled, time-synced mp4.
# Shorter runs freeze on their last frame until the longest run ends.
#
# Usage: tools/compare_videos.sh <label=in.mp4>... <out.mp4>
#   e.g. tools/compare_videos.sh "Pure Pursuit=a.mp4" "Stanley=b.mp4" "LQR=c.mp4" out.mp4
# PANEL_HEIGHT (env, default 480) sets the height every panel is scaled to.
set -euo pipefail

if [ $# -lt 3 ]; then
  sed -n '5,7p' "$0"
  exit 1
fi

output="${!#}"
specs=("${@:1:$#-1}")
panel_height="${PANEL_HEIGHT:-480}"
header_height=$(( panel_height / 12 / 2 * 2 ))
separator=4
font="$(fc-match -f '%{file}' 'sans:bold')"

inputs=()
filters=""
stack_inputs=""
longest=0
for index in "${!specs[@]}"; do
  label="${specs[$index]%%=*}"
  path="${specs[$index]#*=}"
  duration="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$path")"
  longest="$(python3 -c "print(max($longest, $duration))")"
  inputs+=(-i "$path")
  # Label goes in a header bar above the panel so it never covers the video's own overlays;
  # a gray right border separates neighboring panels.
  filters+="[${index}:v]scale=-2:${panel_height},setsar=1,tpad=stop_mode=clone:stop_duration=3600,"
  filters+="pad=iw+${separator}:ih+${header_height}:0:${header_height}:color=0x2b2b2b,"
  filters+="drawtext=fontfile=${font}:text='${label}':fontsize=${header_height}*0.6:fontcolor=white:"
  filters+="x=(w-${separator}-text_w)/2:y=(${header_height}-text_h)/2[panel${index}];"
  stack_inputs+="[panel${index}]"
done
filters+="${stack_inputs}hstack=inputs=${#specs[@]},crop=iw-${separator}:ih:0:0[stacked]"

ffmpeg -hide_banner -loglevel error -y "${inputs[@]}" \
  -filter_complex "$filters" -map "[stacked]" -t "$longest" \
  -c:v libx264 -crf 28 -preset slow -pix_fmt yuv420p -movflags +faststart "$output"

echo "$output: $(du -h "$output" | cut -f1), ${longest}s"

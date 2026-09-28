#!/bin/bash
# Download the race-track centerlines used by navigation.py and trajectory_generator.py.
# Source: TUM racetrack-database (LGPL-3.0), https://github.com/TUMFTM/racetrack-database
# The Silverstone file reproduces tracks/Silverstone_vref_test.npy exactly.
set -euo pipefail
cd "$(dirname "$0")"
for track in Silverstone Monza Suzuka; do
  curl -sfL -o "${track}.csv" \
    "https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/${track}.csv"
  echo "tracks/${track}.csv"
done

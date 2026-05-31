#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

UTILS_FILE="$SCRIPT_DIR/utils.sh"
if [[ ! -f "$UTILS_FILE" ]]; then
  echo "Error: utils.sh not found in $SCRIPT_DIR" >&2
  exit 1
fi
source "$UTILS_FILE"

COMPOSE_FILE="$SCRIPT_DIR/docker/compose/docker-compose_camera_gemini.yml"
if [[ ! -f "$COMPOSE_FILE" ]]; then
  echo "Error: Compose file not found at $COMPOSE_FILE" >&2
  exit 1
fi

main "$COMPOSE_FILE"

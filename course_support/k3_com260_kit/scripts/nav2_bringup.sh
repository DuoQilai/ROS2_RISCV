#!/usr/bin/env bash
set -euo pipefail
script=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/x86-ch22-ch23.bash
args=()
if [[ -f "$HOME/maps/office_map.yaml" ]]; then
  echo "使用地图: $HOME/maps/office_map.yaml"
  args+=(map:=/maps/office_map.yaml)
else
  echo '使用地图: Software_Museum.yaml'
fi
exec bash "$script" start-nav "${1:-ch14}" "${args[@]}"

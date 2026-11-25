#!/usr/bin/env bash
set -euo pipefail
script=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/x86-ch22-ch23.bash
exec bash "$script" start-slam "${1:-ch13}"

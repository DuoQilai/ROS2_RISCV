#!/usr/bin/env bash
set -Eeuo pipefail
export MPLBACKEND=Agg
export PATH="$HOME/ros2_course_com260_ch21_evo/bin:$PATH"
cd "$HOME/ros2_course_com260_ch21_data"
[[ $# == 1 && -r "$1/trajectory-ns.txt" ]] || { echo 'Usage: evaluate.bash RUN_DIRECTORY' >&2; exit 1; }
run=$1
python3 - "$run" <<'CONVERT'
from decimal import Decimal
from pathlib import Path
import sys
folder = Path(sys.argv[1])
with (folder/'trajectory.tum').open('x') as output:
    for line in (folder/'trajectory-ns.txt').read_text().splitlines():
        fields = line.split()
        if fields:
            assert len(fields) == 8
            fields[0] = format(Decimal(fields[0]) / Decimal(10**9), '.9f')
            output.write(' '.join(fields) + '\n')
CONVERT
evo_ape tum reference.tum "$run/trajectory.tum" -a -r trans_part --t_max_diff 0.01 \
  --save_results "$run/ate.zip" --save_plot "$run/ate.png" --plot_mode xyz --no_warnings \
  | tee "$run/ate.log"
evo_rpe tum reference.tum "$run/trajectory.tum" -a -r trans_part --delta 1 --delta_unit f \
  --t_max_diff 0.01 --save_results "$run/rpe.zip" --save_plot "$run/rpe.png" \
  --plot_mode xyz --no_warnings | tee "$run/rpe.log"
evo_traj tum "$run/trajectory.tum" --ref reference.tum -a --t_max_diff 0.01 \
  --save_plot "$run/trajectory.png" --plot_mode xyz --no_warnings | tee "$run/trajectory.log"

"""Compare exported TUM poses with matching simulator timestamps."""
import argparse
import json
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("framework", choices=["dso", "vins"])
args = parser.parse_args()
estimate = np.loadtxt(f"out/{args.framework}/poses.txt", ndmin=2)
truth = np.loadtxt("sequence/truth-body.txt")
indices = np.abs(estimate[:, 0, None] - truth[:, 0]).argmin(axis=1)
if np.max(np.abs(estimate[:, 0]-truth[indices, 0])) > .026:
    raise ValueError("Trajectory timestamps do not match the synthetic sequence.")
x = estimate[:, 1:4]
y = truth[indices, 1:4]
centered_x = x-x.mean(axis=0)
centered_y = y-y.mean(axis=0)
u, singular, vt = np.linalg.svd(centered_x.T @ centered_y)
sign = np.diag([1., 1., np.sign(np.linalg.det(u @ vt))])
rotation = u @ sign @ vt
scale = 1.
if args.framework == "dso":
    scale = np.sum(singular*np.diag(sign)) / np.sum(centered_x**2)
aligned = scale*centered_x @ rotation + y.mean(axis=0)
rmse = float(np.sqrt(np.mean(np.sum((aligned-y)**2, axis=1))))
span = float(estimate[-1, 0]-estimate[0, 0])
passed = bool(np.isfinite(estimate).all() and span >= 22.5 and rmse < .3)
result = dict(poses=len(estimate),span_s=span,scale=float(scale),ate_rmse_m=rmse,passed=passed)
Path(f"out/{args.framework}/evaluation.json").write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps(result, indent=2))
raise SystemExit(0 if passed else 1)

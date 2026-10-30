"""Offline arc-length sampling; the controller reads the resulting external CSV."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tendon_sim.references import load_trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--spacing-mm', type=float, default=10.)
    args = parser.parse_args()
    if not np.isfinite(args.spacing_mm) or args.spacing_mm <= 0:
        parser.error('Spacing must be finite and positive')
    report = args.output.with_suffix('.json')
    if args.output.exists() or report.exists():
        raise FileExistsError('Output CSV and report must be new')
    points = load_trajectory(args.source)
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('At least two finite points required')
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if np.any(np.diff(arc) <= 0):
        raise ValueError('Source must have positive segment lengths')
    stations = np.arange(0., arc[-1], args.spacing_mm / 1000.)
    if arc[-1] - stations[-1] <= 1e-12:
        stations[-1] = arc[-1]
    else:
        stations = np.r_[stations, arc[-1]]
    sampled = np.column_stack([np.interp(stations, arc, points[:, axis]) for axis in range(3)])
    sampled[0], sampled[-1] = points[0], points[-1]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output, np.column_stack([stations, sampled]), delimiter=',', header='s_m,x_m,y_m,z_m', comments='', fmt='%.18e')
    metadata = dict(source=str(args.source), source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
                    output_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(), source_points=len(points),
                    sampled_points=len(sampled), spacing_mm=args.spacing_mm, source_arc_length_m=float(arc[-1]),
                    final_interval_mm=float((stations[-1]-stations[-2])*1000),
                    method='Linear interpolation along original polyline arc; preserve exact endpoints; final interval may be shorter',
                    unit='m', coordinate_frame='world')
    report.write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata))


if __name__ == '__main__':
    main()

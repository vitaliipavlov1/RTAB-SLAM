#!/usr/bin/env python3
"""Offline validation of a finished rtabmap_minimal run.

Everything is read back from what RTAB-Map already saved: the database is opened
read-only and nothing is recomputed - no SLAM, no re-optimization. The checks
look at our integration (D435i -> SensorData -> RTAB-Map) and at how the system
behaved, not at RTAB-Map's algorithms.

    ./scripts/validate_run.py rtabmap_minimal.db
    ./scripts/validate_run.py --traj rtabmap_minimal_trajectory.txt rtabmap_minimal.db
    ./scripts/validate_run.py runs/*.db        # one row per run + spread

Optimized poses come from the exported trajectory when --traj is given, and
otherwise from the last optimization RTAB-Map stored in the database.
Exit code is 1 when a check FAILs, 0 otherwise.
"""

import argparse
import math
import sqlite3
import sys
import zlib
from dataclasses import dataclass

import numpy as np

# Link types, from rtabmap/core/Link.h.
NEIGHBOR, GLOBAL_CLOSURE, PROXIMITY, GRAVITY = 0, 1, 2, 9

# An odometry covariance of 9999 (odometry was reset) arrives as information 1e-4.
WEAK_INFORMATION = 1.0

# First guesses - calibrate them on a few runs of your own route.
GRAVITY_MEAN_DEG = 2.0             # wrong IMU axes would show tens of degrees
GRAVITY_P95_DEG = 5.0
GRAVITY_SPEED_CORRELATION = 0.5    # error growing with rotation speed = stale IMU
LOOP_TRANSLATION_M = 2.0           # a closure beyond the trusted depth range
LOOP_ROTATION_DEG = 60.0
ODOM_DRIFT_RATIO = 0.05            # start->end odometry gap over the travelled path
SLOW_ITERATION_RATIO = 0.01        # share of iterations above Rtabmap/TimeThr


# --- reading the run ---------------------------------------------------------

@dataclass
class Run:
    path: str
    params: dict        # the parameters this run actually used
    stamps: dict        # node id -> timestamp
    map_ids: dict       # node id -> map id
    odom: dict          # node id -> 3x4 odometry pose
    optimized: dict     # node id -> 3x4 optimized pose
    exported: bool      # optimized poses came from the exported trajectory
    links: list         # (type, from, to, 6x6 information, 3x4 transform)
    stats: list         # one dict per RTAB-Map iteration
    calibrations: list  # raw calibration blob, one per node


def _pose(blob):
    """RTAB-Map stores a Transform as 12 raw float32, 3x4 row major."""
    return np.frombuffer(blob, dtype=np.float32).reshape(3, 4).astype(float)


def _information(blob):
    return np.frombuffer(blob, dtype=np.float64).reshape(6, 6)


def _statistics(blob):
    """One iteration of Rtabmap::process, stored zlib-compressed as "key:value;"."""
    values = {}
    for item in zlib.decompress(blob).decode("utf-8", "replace").split(";"):
        key, _, value = item.partition(":")
        try:
            values[key] = float(value)
        except ValueError:
            pass
    return values


def _rotation(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_trajectory(path):
    """The exported file: timestamp x y z qx qy qz qw id."""
    poses = {}
    with open(path) as handle:
        for line in handle:
            if line.startswith("#") or not line.strip():
                continue
            _, x, y, z, qx, qy, qz, qw, node = line.split()
            translation = np.array([[float(x)], [float(y)], [float(z)]])
            rotation = _rotation(float(qx), float(qy), float(qz), float(qw))
            poses[int(node)] = np.hstack([rotation, translation])
    return poses


def _optimized_from_database(connection):
    """Poses of the last optimization, saved when the map was closed."""
    ids, poses = connection.execute("select opt_ids, opt_poses from Admin").fetchone()
    if not ids or not poses:
        return {}
    ids = np.frombuffer(zlib.decompress(ids), dtype=np.int32)
    poses = np.frombuffer(zlib.decompress(poses), dtype=np.float32).reshape(-1, 3, 4)
    return {int(i): pose.astype(float) for i, pose in zip(ids, poses)}


def read_run(path, trajectory_path=None):
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)   # never write to a run
    try:
        stamps, map_ids, odom = {}, {}, {}
        for node, map_id, stamp, pose in connection.execute(
                "select id, map_id, stamp, pose from Node order by id"):
            stamps[node], map_ids[node] = stamp, map_id
            if pose:
                odom[node] = _pose(pose)

        links = [(kind, source, target, _information(information), _pose(transform))
                 for source, target, kind, information, transform in connection.execute(
                     "select from_id, to_id, type, information_matrix, transform from Link")
                 if source < target or kind == GRAVITY]

        stats = [_statistics(data) for (data,) in
                 connection.execute("select data from Statistics order by id")]
        calibrations = [blob for (blob,) in connection.execute("select calibration from Data")]
        text = connection.execute(
            "select parameters from Info order by rowid desc limit 1").fetchone()[0]
        params = dict(item.partition(":")[::2] for item in text.split(";") if ":" in item)

        optimized = read_trajectory(trajectory_path) if trajectory_path \
            else _optimized_from_database(connection)
    finally:
        connection.close()
    return Run(path, params, stamps, map_ids, odom, optimized, bool(trajectory_path),
               links, stats, calibrations)


# --- small helpers -----------------------------------------------------------

def roll_pitch(pose):
    """Degrees in RTAB-Map's base frame: x forward, y left, z up."""
    rotation = pose[:, :3]
    return (math.degrees(math.atan2(rotation[2, 1], rotation[2, 2])),
            math.degrees(math.asin(np.clip(-rotation[2, 0], -1.0, 1.0))))


def rotation_angle(pose):
    """Magnitude of the rotation of a transform, in degrees."""
    cosine = (np.trace(pose[:, :3]) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def positions(poses):
    return [pose[:, 3] for _, pose in sorted(poses.items())]


def path_length(poses):
    points = positions(poses)
    return sum(float(np.linalg.norm(b - a)) for a, b in zip(points, points[1:]))


def start_end_gap(poses):
    """Distance from the first to the last pose - the closing error of a loop route."""
    points = positions(poses)
    return float(np.linalg.norm(points[-1] - points[0])) if len(points) > 1 else 0.0


def gravity_errors(run):
    """Per node: (roll, pitch) of the gravity link minus the optimized pose, in degrees."""
    errors = {}
    for kind, source, _, _, transform in run.links:
        if kind == GRAVITY and source in run.optimized:
            gravity = roll_pitch(transform)
            optimized = roll_pitch(run.optimized[source])
            errors[source] = (gravity[0] - optimized[0], gravity[1] - optimized[1])
    return errors


def links_of(run, kind):
    return [link for link in run.links if link[0] == kind]


def stat_values(run, key, only_positive=True):
    values = [iteration.get(key, 0.0) for iteration in run.stats]
    return [value for value in values if value > 0.0] if only_positive else values


# --- checks ------------------------------------------------------------------
# Each check returns (status, name, detail) with status PASS, WARN, FAIL or SKIP.

def check_export(run):
    """A1: the exported trajectory holds every node, with increasing stamps."""
    if not run.exported:
        return "SKIP", "export", "pass --traj to check the exported trajectory"
    missing = sorted(set(run.stamps) - set(run.optimized))
    stamps = [run.stamps[node] for node in sorted(run.stamps)]
    if missing:
        return "FAIL", "export", f"{len(missing)} of {len(run.stamps)} nodes missing, e.g. {missing[:5]}"
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        return "FAIL", "export", "node timestamps are not strictly increasing"
    return "PASS", "export", f"{len(run.optimized)} poses, one per node, stamps increasing"


def check_map_segments(run):
    """A2: odometry resets split the graph - every segment must survive the export."""
    weak = [(source, target) for kind, source, target, information, _ in run.links
            if kind == NEIGHBOR and information[0, 0] < WEAK_INFORMATION]
    segments = sorted(set(run.map_ids.values()))
    exported = {run.map_ids[node] for node in run.optimized if node in run.map_ids}
    lost = sorted(set(segments) - exported)
    detail = f"{len(segments)} map segment(s), {len(weak)} odometry link(s) with covariance 9999"
    if lost and run.optimized:
        return "FAIL", "map segments", detail + f", segment(s) {lost} absent from the export"
    if weak or len(segments) > 1:
        return "WARN", "map segments", detail + " (expected after a tracking loss)"
    return "PASS", "map segments", detail


def check_gravity_links(run):
    """A3: with IMU on, RTAB-Map should get one gravity constraint per node."""
    gravity, nodes = len(links_of(run, GRAVITY)), len(run.stamps)
    if gravity == 0:
        return "PASS", "gravity links", f"none - run without IMU (Optimizer/GravitySigma="\
                                        f"{run.params.get('Optimizer/GravitySigma', '?')})"
    if gravity != nodes:
        return "WARN", "gravity links", f"{gravity} for {nodes} nodes - IMU was not always ready"
    return "PASS", "gravity links", f"{gravity} for {nodes} nodes"


def check_gravity_agreement(run):
    """A4: IMU orientation against the optimized poses - catches wrong axes or signs."""
    errors = gravity_errors(run)
    if not errors:
        return "SKIP", "IMU agreement", "no gravity links to compare"
    roll = np.array([error[0] for error in errors.values()])
    pitch = np.array([error[1] for error in errors.values()])
    detail = (f"roll mean {roll.mean():+.2f} p95 {np.percentile(abs(roll), 95):.2f}, "
              f"pitch mean {pitch.mean():+.2f} p95 {np.percentile(abs(pitch), 95):.2f} deg")
    if abs(roll.mean()) > GRAVITY_MEAN_DEG or abs(pitch.mean()) > GRAVITY_MEAN_DEG:
        return "FAIL", "IMU agreement", detail + " - check imuLocalTransform axes"
    if max(np.percentile(abs(roll), 95), np.percentile(abs(pitch), 95)) > GRAVITY_P95_DEG:
        return "WARN", "IMU agreement", detail
    return "PASS", "IMU agreement", detail


def check_imu_latency(run):
    """A5: the IMU orientation is the newest one, not the one of the frame. If that
    matters, the error of A4 grows with how fast the camera was turning."""
    errors = gravity_errors(run)
    speeds, deviations = [], []
    nodes = sorted(errors)
    for previous, node in zip(nodes, nodes[1:]):
        seconds = run.stamps[node] - run.stamps[previous]
        if seconds > 0 and previous in run.odom and node in run.odom:
            turn = run.odom[previous][:, :3].T @ run.odom[node][:, :3]
            speeds.append(rotation_angle(np.hstack([turn, np.zeros((3, 1))])) / seconds)
            deviations.append(math.hypot(*errors[node]))
    if len(speeds) < 10:
        return "SKIP", "IMU latency", "not enough motion to correlate"
    correlation = float(np.corrcoef(speeds, deviations)[0, 1])
    detail = f"error vs rotation speed correlation {correlation:+.2f}"
    if correlation > GRAVITY_SPEED_CORRELATION:
        return "WARN", "IMU latency", detail + " - IMU orientation looks stale on fast turns"
    return "PASS", "IMU latency", detail


def check_loop_closures(run):
    """A6: accepted closures must respect the inlier threshold the run was given."""
    accepted = stat_values(run, "Loop/Accepted_hypothesis_id/")
    rejected = stat_values(run, "Loop/RejectedHypothesis/")
    proximity = stat_values(run, "Proximity/Space_last_detection_id/")
    minimum = float(run.params.get("Vis/MinInliers", 0))
    inliers = [iteration["Loop/Visual_inliers/"] for iteration in run.stats
               if iteration.get("Loop/Id/", 0.0) > 0 and iteration.get("Loop/Visual_inliers/", 0.0) > 0]
    detail = f"{len(accepted)} accepted, {len(proximity)} proximity, {len(rejected)} rejected"
    if inliers:
        detail += f"; inliers min {min(inliers):.0f}"
    if inliers and min(inliers) < minimum:
        return "FAIL", "loop closures", detail + f" below Vis/MinInliers={minimum:.0f}"
    if not accepted and not proximity:
        return "WARN", "loop closures", detail + " - the route never closed"
    return "PASS", "loop closures", detail


def check_loop_plausibility(run):
    """A7: a closure linking poses farther apart than the camera can see is suspect."""
    suspicious = []
    for kind, source, target, _, transform in run.links:
        if kind not in (GLOBAL_CLOSURE, PROXIMITY):
            continue
        distance = float(np.linalg.norm(transform[:, 3]))
        angle = rotation_angle(transform)
        if distance > LOOP_TRANSLATION_M or angle > LOOP_ROTATION_DEG:
            suspicious.append(f"{source}->{target} {distance:.2f} m {angle:.0f} deg")
    total = len(links_of(run, GLOBAL_CLOSURE)) + len(links_of(run, PROXIMITY))
    if suspicious:
        return "WARN", "closure plausibility", f"{len(suspicious)} of {total}: {suspicious[:3]}"
    return "PASS", "closure plausibility", f"all {total} closures within "\
                                           f"{LOOP_TRANSLATION_M} m / {LOOP_ROTATION_DEG} deg"


def check_drift(run):
    """A8: how much the graph had to correct the raw odometry."""
    if not run.optimized:
        return "SKIP", "drift", "no optimized poses"
    shared = sorted(set(run.odom) & set(run.optimized))
    if len(shared) < 2:
        return "SKIP", "drift", "not enough poses in common"
    corrections = [float(np.linalg.norm(run.odom[node][:, 3] - run.optimized[node][:, 3]))
                   for node in shared]
    travelled = path_length(run.optimized)
    odometry_gap = start_end_gap({node: run.odom[node] for node in shared})
    detail = (f"path {travelled:.2f} m, start->end {start_end_gap(run.optimized):.2f} m "
              f"optimized vs {odometry_gap:.2f} m odometry, "
              f"max correction {max(corrections):.2f} m")
    if travelled > 0 and odometry_gap / travelled > ODOM_DRIFT_RATIO:
        return "WARN", "drift", detail
    return "PASS", "drift", detail


def check_covariances(run):
    """A9: the covariance we hand to RTAB-Map must stay usable."""
    diagonals = [np.diag(information) for kind, _, _, information, _ in run.links
                 if kind == NEIGHBOR]
    if not diagonals:
        return "SKIP", "covariances", "no odometry links"
    diagonals = np.array(diagonals)
    if not np.isfinite(diagonals).all() or (diagonals <= 0).any():
        return "FAIL", "covariances", "odometry links with zero, negative or non-finite information"
    return "PASS", "covariances", f"linear information median {np.median(diagonals[:, 0]):.0f}, "\
                                  f"min {diagonals[:, 0].min():.4f}"


def check_timing(run):
    """A10: RTAB-Map must keep up with Rtabmap/DetectionRate."""
    totals = stat_values(run, "Timing/Total/ms")
    if not totals:
        return "SKIP", "timing", "no timing statistics"
    median, p95, worst = np.median(totals), np.percentile(totals, 95), max(totals)
    threshold = float(run.params.get("Rtabmap/TimeThr", 0))
    slow = sum(value > threshold for value in totals) if threshold else 0
    detail = f"p50 {median:.0f} p95 {p95:.0f} max {worst:.0f} ms, "\
             f"{slow}/{len(totals)} above Rtabmap/TimeThr={threshold:.0f}"
    budget = 1000.0 / float(run.params.get("Rtabmap/DetectionRate", 1.0) or 1.0)
    if p95 > budget or (threshold and slow > SLOW_ITERATION_RATIO * len(totals)):
        return "WARN", "timing", detail
    return "PASS", "timing", detail


def check_calibration(run):
    """A11: one camera, one calibration - it must not change mid-run."""
    distinct = {bytes(blob) for blob in run.calibrations if blob}
    if not distinct:
        return "SKIP", "calibration", "no calibration stored"
    if len(distinct) > 1:
        return "FAIL", "calibration", f"{len(distinct)} different calibrations in one run"
    return "PASS", "calibration", f"identical in all {len(run.calibrations)} nodes"


CHECKS = [check_export, check_map_segments, check_gravity_links, check_gravity_agreement,
          check_imu_latency, check_loop_closures, check_loop_plausibility, check_drift,
          check_covariances, check_timing, check_calibration]


# --- output ------------------------------------------------------------------

def print_run(run):
    seconds = max(run.stamps.values()) - min(run.stamps.values()) if run.stamps else 0.0
    travelled = run.stats[-1].get("Memory/Distance_travelled/m", 0.0) if run.stats else 0.0
    print(f"\n=== {run.path}")
    print(f"nodes {len(run.stamps)} | {seconds:.0f} s | travelled {travelled:.1f} m | "
          f"optimized poses from {'trajectory file' if run.exported else 'database'}")
    print(f"links: {len(links_of(run, NEIGHBOR))} odometry, "
          f"{len(links_of(run, GLOBAL_CLOSURE))} loop closure, "
          f"{len(links_of(run, PROXIMITY))} proximity, {len(links_of(run, GRAVITY))} gravity")

    failed = False
    for check in CHECKS:
        status, name, detail = check(run)
        failed = failed or status == "FAIL"
        print(f"  {status:4} {name:22} {detail}")
    return failed


def print_comparison(runs):
    """T7 and T8: the same route several times, or with and without IMU."""
    print("\n=== comparison")
    print(f"  {'run':28} {'nodes':>6} {'path m':>8} {'gap m':>7} {'closures':>9} {'gravity':>8}")
    lengths, gaps = [], []
    for run in runs:
        lengths.append(path_length(run.optimized))
        gaps.append(start_end_gap(run.optimized))
        print(f"  {run.path[-28:]:28} {len(run.stamps):6} {lengths[-1]:8.2f} {gaps[-1]:7.2f} "
              f"{len(links_of(run, GLOBAL_CLOSURE)):9} {len(links_of(run, GRAVITY)):8}")
    if len(runs) > 1 and np.mean(lengths) > 0:
        print(f"  path length spread: {100 * np.std(lengths) / np.mean(lengths):.1f} % of mean")
        print(f"  start->end gap: median {np.median(gaps):.2f} m, max {max(gaps):.2f} m")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("databases", nargs="+", help="rtabmap_minimal .db file(s)")
    parser.add_argument("--traj", help="exported trajectory of a single run")
    arguments = parser.parse_args()
    if arguments.traj and len(arguments.databases) > 1:
        parser.error("--traj describes one run, so pass one database")

    runs = [read_run(path, arguments.traj) for path in arguments.databases]
    failed = [print_run(run) for run in runs]
    if len(runs) > 1:
        print_comparison(runs)
    return 1 if any(failed) else 0


if __name__ == "__main__":
    sys.exit(main())

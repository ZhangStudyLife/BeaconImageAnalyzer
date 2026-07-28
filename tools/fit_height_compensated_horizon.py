#!/usr/bin/env python3
"""Fit a two-parameter height compensation on top of a fixed fisheye model."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


SEARCH_RADIUS_PX = 8
MIN_HEIGHT_MM = 650.0
MIN_RIDGE_VALUE = 18.0
MIN_RIDGE_RESPONSE = 5.0
MIN_MEDIAN_RESPONSE = 6.0
MIN_RUN_POINTS = 25
MIN_RUN_SPAN = 25
FRAME_INLIER_RMSE_PX = 2.0
RANSAC_ITERATIONS = 500
RANSAC_SEED = 0x4843414C
DISTANCE_RANGE_MM = (6000.0, 8000.0)
HEIGHT_ZERO_RANGE_MM = (800.0, 1400.0)


@dataclass
class Model:
    source: Path
    root: dict
    width: int
    height: int
    camera_id: int
    center_x: float
    center_y: float
    scale: float
    theta: np.ndarray
    matrix: np.ndarray
    rays: np.ndarray


@dataclass
class CandidateFrame:
    frame_index: int
    sequence: int
    roll_deg: float
    pitch_deg: float
    height_mm: float
    points: np.ndarray
    responses: np.ndarray


@dataclass
class PointTable:
    frame_ids: np.ndarray
    base: np.ndarray
    term: np.ndarray
    dx_base: np.ndarray
    dx_term: np.ndarray
    dy_base: np.ndarray
    dy_term: np.ndarray
    frame_counts: np.ndarray


def load_model(path: Path) -> Model:
    root = json.loads(path.read_text(encoding="utf-8"))
    if (root.get("format") != "horizon_model"
            or root.get("version") != 3
            or root.get("model_type") != "central_fisheye_angle_poly5"):
        raise ValueError("基础模型必须是v3 central_fisheye_angle_poly5物理鱼眼模型")
    width = int(root["image_width"])
    height = int(root["image_height"])
    center_x = float(root["center_x"])
    center_y = float(root["center_y"])
    scale = float(root["normalization_scale"])
    theta = np.asarray(root["theta_coefficients"], dtype=np.float64)
    matrix = np.asarray(root["attitude_to_camera_normal"], dtype=np.float64)
    if theta.shape != (3,) or matrix.shape != (3, 3):
        raise ValueError("基础模型的鱼眼参数或姿态矩阵尺寸无效")
    yy, xx = np.mgrid[0:height, 0:width]
    rays = rays_for_coordinates(xx, yy, center_x, center_y, scale, theta)
    return Model(path.resolve(), root, width, height, int(root["camera_id"]),
                 center_x, center_y, scale, theta, matrix, rays)


def rays_for_coordinates(x, y, center_x, center_y, scale, theta):
    xn = (np.asarray(x, dtype=np.float64) - center_x) / scale
    yn = (np.asarray(y, dtype=np.float64) - center_y) / scale
    radius2 = xn * xn + yn * yn
    radius = np.sqrt(radius2)
    angle = radius * (theta[0] + radius2 * (theta[1] + radius2 * theta[2]))
    radial = np.divide(np.sin(angle), radius,
                       out=np.full_like(radius, theta[0]), where=radius > 1e-12)
    return np.stack((xn * radial, yn * radial, np.cos(angle)), axis=-1)


def ray_at(model: Model, x: float, y: float) -> np.ndarray:
    return rays_for_coordinates(x, y, model.center_x, model.center_y,
                                model.scale, model.theta)


def attitude_axes(roll_deg: float, pitch_deg: float) -> tuple[np.ndarray, np.ndarray]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    gravity = np.array((-sp, sr * cp, cr * cp), dtype=np.float64)
    forward = np.array((cp, sr * sp, cr * sp), dtype=np.float64)
    return gravity, forward


def direction(model: Model, roll_deg: float, pitch_deg: float,
              height_mm: float | None = None,
              parameters: np.ndarray | None = None) -> np.ndarray:
    gravity, forward = attitude_axes(roll_deg, pitch_deg)
    if height_mm is not None and parameters is not None:
        gravity = gravity + (parameters[0] * height_mm + parameters[1]) * forward
    return model.matrix @ gravity


def roots_at_x(values: np.ndarray) -> list[float]:
    indices = np.flatnonzero(values[:-1] * values[1:] < 0.0)
    roots = [float(index + abs(values[index])
                   / (abs(values[index]) + abs(values[index + 1])))
             for index in indices]
    exact = np.flatnonzero(np.abs(values) < 1e-12)
    for index in exact:
        if not roots or min(abs(root - index) for root in roots) > 0.5:
            roots.append(float(index))
    roots.sort()
    return roots


def predict_curve(model: Model, roll_deg: float, pitch_deg: float,
                  height_mm: float | None = None,
                  parameters: np.ndarray | None = None) -> np.ndarray:
    values = np.tensordot(model.rays,
                          direction(model, roll_deg, pitch_deg, height_mm, parameters),
                          axes=([2], [0]))
    roots = [roots_at_x(values[:, x]) for x in range(model.width)]
    center = int(np.clip(round(model.center_x), 0, model.width - 1))
    seed = -1
    for offset in range(model.width):
        for x in (center - offset, center + offset):
            if 0 <= x < model.width and roots[x]:
                seed = x
                break
        if seed >= 0:
            break
    curve = np.full(model.width, np.nan, dtype=np.float64)
    if seed < 0:
        return curve
    curve[seed] = min(roots[seed], key=lambda y: abs(y - model.center_y))
    previous = curve[seed]
    for x in range(seed - 1, -1, -1):
        if not roots[x]:
            break
        previous = min(roots[x], key=lambda y: abs(y - previous))
        curve[x] = previous
    previous = curve[seed]
    for x in range(seed + 1, model.width):
        if not roots[x]:
            break
        previous = min(roots[x], key=lambda y: abs(y - previous))
        curve[x] = previous
    return curve


def extract_ridge(gray: np.ndarray, curve: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    candidates = []
    for x, predicted_y in enumerate(curve):
        if not math.isfinite(predicted_y):
            continue
        center_y = int(round(predicted_y))
        best = None
        for offset in range(-SEARCH_RADIUS_PX, SEARCH_RADIUS_PX + 1):
            y = center_y + offset
            if y < 4 or y >= gray.shape[0] - 4:
                continue
            value = float(gray[y, x])
            response = value - 0.5 * (float(gray[y - 3, x]) + float(gray[y + 3, x]))
            if best is None or response > best[0]:
                best = response, value, y
        if best is not None and best[0] >= MIN_RIDGE_RESPONSE and best[1] >= MIN_RIDGE_VALUE:
            candidates.append((x, best[2], best[0]))

    runs: list[list[tuple[int, int, float]]] = []
    run: list[tuple[int, int, float]] = []
    for point in candidates:
        if run and (point[0] - run[-1][0] > 3 or abs(point[1] - run[-1][1]) > 4):
            runs.append(run)
            run = []
        run.append(point)
    if run:
        runs.append(run)
    if not runs:
        return np.empty((0, 2), dtype=np.float64), np.empty(0, dtype=np.float64)
    best_run = max(runs, key=lambda points: (points[-1][0] - points[0][0], len(points)))
    points = np.asarray([(x, y) for x, y, _ in best_run], dtype=np.float64)
    responses = np.asarray([response for _, _, response in best_run], dtype=np.float64)
    if (len(points) < MIN_RUN_POINTS
            or points[-1, 0] - points[0, 0] < MIN_RUN_SPAN
            or np.median(responses) < MIN_MEDIAN_RESPONSE):
        return np.empty((0, 2), dtype=np.float64), np.empty(0, dtype=np.float64)
    return points, responses


def read_metadata(csv_path: Path) -> list[dict]:
    with csv_path.open("r", encoding="utf-8-sig", newline="") as file:
        rows = list(csv.DictReader(file))
    required = {"frame_index", "bimg_sequence", "camera_id", "roll_deg", "pitch_deg",
                "height_mm", "attitude_valid", "height_valid"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError("HCAL CSV不是包含高度的v3格式")
    return rows


def extract_candidates(model: Model, session: dict, session_path: Path,
                       stride: int) -> tuple[list[CandidateFrame], Path, Path]:
    directory = session_path.parent
    csv_path = (directory / session["csv_file"]).resolve()
    video_path = (directory / session["video_file"]).resolve()
    rows = read_metadata(csv_path)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"无法打开视频：{video_path}")
    candidates = []
    decoded = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if decoded >= len(rows):
            break
        row = rows[decoded]
        if (decoded % stride == 0 and row["attitude_valid"] == "1"
                and row["height_valid"] == "1"):
            height_mm = float(row["height_mm"])
            if height_mm >= MIN_HEIGHT_MM:
                roll_deg = float(row["roll_deg"])
                pitch_deg = float(row["pitch_deg"])
                gray = (cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        if frame.ndim == 3 else frame)
                points, responses = extract_ridge(
                    gray, predict_curve(model, roll_deg, pitch_deg))
                if len(points):
                    candidates.append(CandidateFrame(
                        decoded, int(row["bimg_sequence"]), roll_deg, pitch_deg,
                        height_mm, points, responses))
        decoded += 1
    capture.release()
    if decoded != len(rows):
        raise RuntimeError(f"视频帧数{decoded}与CSV行数{len(rows)}不一致")
    return candidates, csv_path, video_path


def build_point_table(model: Model, frames: list[CandidateFrame]) -> PointTable:
    frame_ids = []
    base = []
    term = []
    dx_base = []
    dx_term = []
    dy_base = []
    dy_term = []
    counts = []
    epsilon = 0.25
    for frame_id, frame in enumerate(frames):
        gravity, forward = attitude_axes(frame.roll_deg, frame.pitch_deg)
        mapped_gravity = model.matrix @ gravity
        mapped_forward = model.matrix @ forward
        selected = frame.points[::3]
        counts.append(len(selected))
        for x, y in selected:
            ray = ray_at(model, x, y)
            ray_dx = (ray_at(model, x + epsilon, y) - ray_at(model, x - epsilon, y)) \
                     / (2.0 * epsilon)
            ray_dy = (ray_at(model, x, y + epsilon) - ray_at(model, x, y - epsilon)) \
                     / (2.0 * epsilon)
            frame_ids.append(frame_id)
            base.append(float(ray @ mapped_gravity))
            term.append(float(ray @ mapped_forward))
            dx_base.append(float(ray_dx @ mapped_gravity))
            dx_term.append(float(ray_dx @ mapped_forward))
            dy_base.append(float(ray_dy @ mapped_gravity))
            dy_term.append(float(ray_dy @ mapped_forward))
    return PointTable(np.asarray(frame_ids, dtype=np.int32),
                      np.asarray(base), np.asarray(term), np.asarray(dx_base),
                      np.asarray(dx_term), np.asarray(dy_base), np.asarray(dy_term),
                      np.asarray(counts, dtype=np.int32))


def solve_parameters(table: PointTable, frames: list[CandidateFrame],
                     frame_mask: np.ndarray, frame_weights: np.ndarray | None = None) -> np.ndarray:
    point_mask = frame_mask[table.frame_ids]
    ids = table.frame_ids[point_mask]
    heights = np.asarray([frame.height_mm for frame in frames])[ids]
    matrix = np.column_stack((table.term[point_mask] * heights,
                              table.term[point_mask]))
    target = -table.base[point_mask]
    if frame_weights is None:
        weights = 1.0 / table.frame_counts[ids]
    else:
        weights = frame_weights[ids] / table.frame_counts[ids]
    weighted = np.sqrt(np.maximum(weights, 1e-12))
    return np.linalg.lstsq(matrix * weighted[:, None], target * weighted, rcond=None)[0]


def physical_values(parameters: np.ndarray) -> tuple[float, float]:
    alpha, beta = parameters
    if alpha >= 0.0 or not np.all(np.isfinite(parameters)):
        return math.nan, math.nan
    distance = -1.0 / alpha
    height_zero = -beta / alpha
    return distance, height_zero


def parameters_valid(parameters: np.ndarray) -> bool:
    distance, height_zero = physical_values(parameters)
    return (DISTANCE_RANGE_MM[0] <= distance <= DISTANCE_RANGE_MM[1]
            and HEIGHT_ZERO_RANGE_MM[0] <= height_zero <= HEIGHT_ZERO_RANGE_MM[1])


def point_errors(table: PointTable, frames: list[CandidateFrame],
                 parameters: np.ndarray) -> np.ndarray:
    heights = np.asarray([frame.height_mm for frame in frames])[table.frame_ids]
    factor = parameters[0] * heights + parameters[1]
    value = table.base + table.term * factor
    derivative_x = table.dx_base + table.dx_term * factor
    derivative_y = table.dy_base + table.dy_term * factor
    denominator = np.hypot(derivative_x, derivative_y)
    return np.divide(np.abs(value), denominator,
                     out=np.full_like(value, 1e6), where=denominator > 1e-12)


def frame_rmse(table: PointTable, frames: list[CandidateFrame],
               parameters: np.ndarray) -> np.ndarray:
    errors = point_errors(table, frames, parameters)
    sums = np.bincount(table.frame_ids, weights=errors * errors, minlength=len(frames))
    return np.sqrt(sums / table.frame_counts)


def ransac_fit(table: PointTable, frames: list[CandidateFrame], iterations: int,
               allowed_frames: np.ndarray | None = None, seed: int = RANSAC_SEED):
    allowed = (np.ones(len(frames), dtype=bool) if allowed_frames is None
               else allowed_frames.copy())
    indices = np.flatnonzero(allowed)
    if len(indices) < 2:
        raise RuntimeError("可用于拟合的候选帧不足")
    rng = random.Random(seed)
    best_parameters = None
    best_inliers = None
    best_score = (-1, -math.inf, -math.inf)
    heights = np.asarray([frame.height_mm for frame in frames])
    for _ in range(iterations):
        first, second = rng.sample(list(indices), 2)
        if abs(heights[first] - heights[second]) < 80.0:
            continue
        sample = np.zeros(len(frames), dtype=bool)
        sample[[first, second]] = True
        parameters = solve_parameters(table, frames, sample)
        if not parameters_valid(parameters):
            continue
        errors = frame_rmse(table, frames, parameters)
        inliers = allowed & (errors <= FRAME_INLIER_RMSE_PX)
        count = int(np.count_nonzero(inliers))
        if count < 2:
            continue
        score = (count, -float(np.median(errors[inliers])), -float(np.mean(errors[inliers])))
        if score > best_score:
            best_score = score
            best_parameters = parameters
            best_inliers = inliers
    if best_parameters is None:
        raise RuntimeError("RANSAC未找到满足物理范围的高度补偿参数")

    inliers = best_inliers
    parameters = best_parameters
    for _ in range(8):
        errors = frame_rmse(table, frames, parameters)
        huber = np.ones(len(frames), dtype=np.float64)
        active_errors = errors[inliers]
        scale = 1.4826 * np.median(np.abs(active_errors - np.median(active_errors))) + 1e-6
        threshold = max(0.75, 1.5 * scale)
        large = errors > threshold
        huber[large] = threshold / errors[large]
        parameters = solve_parameters(table, frames, inliers, huber)
        if not parameters_valid(parameters):
            parameters = best_parameters
            break
        inliers = allowed & (frame_rmse(table, frames, parameters) <= FRAME_INLIER_RMSE_PX)
    return parameters, inliers, frame_rmse(table, frames, parameters)


def error_metrics(errors: np.ndarray) -> dict:
    if not len(errors):
        return {"count": 0, "rmse_px": None, "median_error_px": None,
                "p90_error_px": None, "p95_error_px": None, "max_error_px": None}
    return {"count": int(len(errors)),
            "rmse_px": float(math.sqrt(np.mean(errors * errors))),
            "median_error_px": float(np.median(errors)),
            "p90_error_px": float(np.percentile(errors, 90)),
            "p95_error_px": float(np.percentile(errors, 95)),
            "max_error_px": float(np.max(errors))}


def cross_validate(table: PointTable, frames: list[CandidateFrame], total_frames: int) -> dict:
    point_errors_all = []
    frame_errors_all = []
    details = []
    frame_indices = np.asarray([frame.frame_index for frame in frames])
    folds = np.minimum(4, frame_indices * 5 // max(total_frames, 1))
    for fold in range(5):
        train = folds != fold
        test = folds == fold
        if np.count_nonzero(train) < 4 or np.count_nonzero(test) == 0:
            continue
        parameters, _, _ = ransac_fit(table, frames, 200, train, RANSAC_SEED + fold + 1)
        test_points = test[table.frame_ids]
        point_fold = point_errors(table, frames, parameters)[test_points]
        frame_fold = frame_rmse(table, frames, parameters)[test]
        point_errors_all.extend(point_fold.tolist())
        frame_errors_all.extend(frame_fold.tolist())
        details.append({"fold": fold, "test_frames": int(np.count_nonzero(test)),
                        "point_metrics": error_metrics(point_fold),
                        "frame_metrics": error_metrics(frame_fold)})
    return {"folds": details,
            "point_metrics": error_metrics(np.asarray(point_errors_all)),
            "frame_metrics": error_metrics(np.asarray(frame_errors_all))}


def sample_annotation(points: np.ndarray, maximum: int = 21) -> list[list[float]]:
    if len(points) <= maximum:
        selected = points
    else:
        selected = points[np.linspace(0, len(points) - 1, maximum, dtype=int)]
    return [[float(x), float(y)] for x, y in selected]


def output_paths(session_path: Path, prefix: Path | None) -> dict[str, Path]:
    if prefix is None:
        name = session_path.name
        if name.endswith(".hcal.json"):
            name = name[:-10]
        prefix = session_path.with_name(name)
    return {"session": Path(str(prefix) + "_height_compensated.hcal.json"),
            "model": Path(str(prefix) + "_height_compensated_model.json"),
            "header": Path(str(prefix) + "_height_compensated_model.h"),
            "overlay": Path(str(prefix) + "_height_compensated_overlay.avi"),
            "report": Path(str(prefix) + "_height_compensated_report.json")}


def write_session(source: dict, path: Path, frames: list[CandidateFrame], inliers: np.ndarray,
                  model_path: Path):
    root = dict(source)
    root["annotations"] = [
        {"frame_index": frame.frame_index, "skipped": False,
         "annotation_type": "curve_points", "points": sample_annotation(frame.points)}
        for index, frame in enumerate(frames) if inliers[index]
    ]
    root["height_fit"] = {"model_file": model_path.name,
                          "inlier_frames": int(np.count_nonzero(inliers)),
                          "candidate_frames": len(frames)}
    path.write_text(json.dumps(root, ensure_ascii=False, indent=4), encoding="utf-8")


def write_model(model: Model, path: Path, parameters: np.ndarray,
                frames: list[CandidateFrame], inliers: np.ndarray,
                metrics: dict, cross_validation: dict, source_session: Path):
    distance, height_zero = physical_values(parameters)
    selected = [frame for index, frame in enumerate(frames) if inliers[index]]
    heights = np.asarray([frame.height_mm for frame in selected])
    rolls = np.asarray([frame.roll_deg for frame in selected])
    pitches = np.asarray([frame.pitch_deg for frame in selected])
    root = dict(model.root)
    root.pop("optimizer_evaluations", None)
    root.update({"version": 4,
                 "model_type": "central_fisheye_height_compensated",
                 "base_model_file": model.source.name,
                 "source_session": str(source_session.resolve()),
                 "effective_distance_mm": distance,
                 "height_zero_mm": height_zero,
                 "height_min_mm": float(np.min(heights)),
                 "height_max_mm": float(np.max(heights)),
                 "roll_min_deg": float(np.min(rolls)),
                 "roll_max_deg": float(np.max(rolls)),
                 "pitch_min_deg": float(np.min(pitches)),
                 "pitch_max_deg": float(np.max(pitches)),
                 "sample_count": len(frames),
                 "inlier_count": len(selected),
                 "rmse_px": metrics["rmse_px"],
                 "median_error_px": metrics["median_error_px"],
                 "max_error_px": metrics["max_error_px"],
                 "height_compensation": {"formula": "q=(height_zero_mm-height_mm)/effective_distance_mm",
                                           "alpha_per_mm": float(parameters[0]),
                                           "beta": float(parameters[1])},
                 "cross_validation": cross_validation})
    path.write_text(json.dumps(root, ensure_ascii=False, indent=4), encoding="utf-8")


def c_identifier(text: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]", "_", text).strip("_").lower() or "front"


def write_header(model: Model, path: Path, parameters: np.ndarray):
    distance, height_zero = physical_values(parameters)
    name = c_identifier(model.root.get("camera_name", "front"))
    guard = f"HORIZON_{name.upper()}_HEIGHT_COMPENSATED_MODEL_H"
    theta = ", ".join(f"{value:.10g}f" for value in model.theta)
    matrix = ",\n    ".join(", ".join(f"{value:.10g}f" for value in row)
                              for row in model.matrix)
    source = f"""#ifndef {guard}
#define {guard}

#include <math.h>

#define HORIZON_{name.upper()}_WIDTH ({model.width})
#define HORIZON_{name.upper()}_HEIGHT ({model.height})

static const float g_horizon_{name}_center_x = {model.center_x:.10g}f;
static const float g_horizon_{name}_center_y = {model.center_y:.10g}f;
static const float g_horizon_{name}_scale = {model.scale:.10g}f;
static const float g_horizon_{name}_effective_distance_mm = {distance:.10g}f;
static const float g_horizon_{name}_height_zero_mm = {height_zero:.10g}f;
static const float g_horizon_{name}_theta[3] = {{{theta}}};
static const float g_horizon_{name}_matrix[3][3] = {{
    {matrix}
}};

static inline void horizon_{name}_height_normal(float roll_deg,
                                                 float pitch_deg,
                                                 float height_mm,
                                                 float normal[3])
{{
    const float rad = 0.01745329251994329577f;
    const float roll = roll_deg * rad;
    const float pitch = pitch_deg * rad;
    const float sr = sinf(roll);
    const float cr = cosf(roll);
    const float sp = sinf(pitch);
    const float cp = cosf(pitch);
    const float q = (g_horizon_{name}_height_zero_mm - height_mm)
                    / g_horizon_{name}_effective_distance_mm;
    const float vector[3] = {{-sp + q * cp,
                              sr * cp + q * sr * sp,
                              cr * cp + q * cr * sp}};
    for (int row = 0; row < 3; ++row)
    {{
        normal[row] = g_horizon_{name}_matrix[row][0] * vector[0]
                      + g_horizon_{name}_matrix[row][1] * vector[1]
                      + g_horizon_{name}_matrix[row][2] * vector[2];
    }}
}}

static inline float horizon_{name}_height_value(float x,
                                                float y,
                                                const float normal[3])
{{
    const float xn = (x - g_horizon_{name}_center_x) / g_horizon_{name}_scale;
    const float yn = (y - g_horizon_{name}_center_y) / g_horizon_{name}_scale;
    const float radius2 = xn * xn + yn * yn;
    const float radius = sqrtf(radius2);
    const float theta = radius * (g_horizon_{name}_theta[0]
                        + radius2 * (g_horizon_{name}_theta[1]
                        + radius2 * g_horizon_{name}_theta[2]));
    const float radial = radius > 1e-6f ? sinf(theta) / radius
                                       : g_horizon_{name}_theta[0];
    return xn * radial * normal[0] + yn * radial * normal[1]
           + cosf(theta) * normal[2];
}}

#endif
"""
    path.write_text(source, encoding="utf-8")


def draw_curve(image: np.ndarray, curve: np.ndarray, color: tuple[int, int, int]):
    previous = None
    for x, y in enumerate(curve):
        if not math.isfinite(y):
            previous = None
            continue
        point = (x, int(round(y)))
        if previous is not None and point[0] == previous[0] + 1:
            cv2.line(image, previous, point, color, 1, cv2.LINE_AA)
        previous = point


def write_overlay(model: Model, video_path: Path, path: Path,
                  metadata: list[dict], parameters: np.ndarray,
                  frames: list[CandidateFrame], inliers: np.ndarray):
    annotations = {frame.frame_index: frame.points for index, frame in enumerate(frames)
                   if inliers[index]}
    capture = cv2.VideoCapture(str(video_path))
    fps = capture.get(cv2.CAP_PROP_FPS) or 50.0
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps,
                             (model.width, model.height), True)
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"无法创建覆盖视频：{path}")
    index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        row = metadata[index]
        if row["attitude_valid"] == "1" and row["height_valid"] == "1":
            curve = predict_curve(model, float(row["roll_deg"]), float(row["pitch_deg"]),
                                  float(row["height_mm"]), parameters)
            draw_curve(frame, curve, (0, 255, 0))
        if index in annotations:
            for x, y in annotations[index]:
                cv2.circle(frame, (int(round(x)), int(round(y))), 1, (255, 0, 255), -1)
        writer.write(frame)
        index += 1
    writer.release()
    capture.release()


def run(args) -> dict:
    session_path = args.session.resolve()
    source_session = json.loads(session_path.read_text(encoding="utf-8"))
    if (source_session.get("format") != "horizon_calibration"
            or source_session.get("version") != 3
            or not source_session.get("height_recorded")):
        raise ValueError("输入必须是包含高度的HCAL v3会话")
    model = load_model(args.base_model.resolve())
    if int(source_session["camera_id"]) != model.camera_id:
        raise ValueError("HCAL相机ID与基础模型不一致")
    candidates, csv_path, video_path = extract_candidates(
        model, source_session, session_path, args.stride)
    if len(candidates) < 20:
        raise RuntimeError(f"自动提取的候选帧不足：{len(candidates)}")
    table = build_point_table(model, candidates)
    parameters, inliers, rmses = ransac_fit(table, candidates, RANSAC_ITERATIONS)
    if np.count_nonzero(inliers) < 12:
        raise RuntimeError("高度模型内点不足12帧")
    points_inlier = inliers[table.frame_ids]
    final_point_errors = point_errors(table, candidates, parameters)[points_inlier]
    final_metrics = error_metrics(final_point_errors)
    baseline_metrics = error_metrics(point_errors(table, candidates, np.zeros(2)))
    cross_validation = cross_validate(table, candidates, int(source_session["frame_count"]))
    outputs = output_paths(session_path, args.output_prefix)
    write_session(source_session, outputs["session"], candidates, inliers, outputs["model"])
    write_model(model, outputs["model"], parameters, candidates, inliers,
                final_metrics, cross_validation, session_path)
    write_header(model, outputs["header"], parameters)
    metadata = read_metadata(csv_path)
    write_overlay(model, video_path, outputs["overlay"], metadata,
                  parameters, candidates, inliers)
    distance, height_zero = physical_values(parameters)
    report = {"source_session": str(session_path),
              "base_model": str(model.source),
              "decoded_frames": int(source_session["frame_count"]),
              "candidate_frames": len(candidates),
              "inlier_frames": int(np.count_nonzero(inliers)),
              "outlier_frames": int(len(candidates) - np.count_nonzero(inliers)),
              "effective_distance_mm": distance,
              "height_zero_mm": height_zero,
              "baseline_point_metrics": baseline_metrics,
              "height_model_point_metrics": final_metrics,
              "inlier_frame_metrics": error_metrics(rmses[inliers]),
              "cross_validation": cross_validation,
              "extraction": {"stride": args.stride,
                             "search_radius_px": SEARCH_RADIUS_PX,
                             "minimum_height_mm": MIN_HEIGHT_MM,
                             "minimum_run_points": MIN_RUN_POINTS,
                             "minimum_run_span": MIN_RUN_SPAN,
                             "frame_inlier_rmse_px": FRAME_INLIER_RMSE_PX,
                             "ransac_iterations": RANSAC_ITERATIONS},
              "outputs": {key: str(value.resolve()) for key, value in outputs.items()}}
    outputs["report"].write_text(json.dumps(report, ensure_ascii=False, indent=4),
                                 encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path, help="HCAL v3 JSON")
    parser.add_argument("--base-model", required=True, type=Path,
                        help="昨日central_fisheye_angle_poly5模型")
    parser.add_argument("--output-prefix", type=Path,
                        help="输出路径前缀，默认与HCAL同名")
    parser.add_argument("--stride", type=int, default=1,
                        help="提取时的帧步长，默认处理每一帧")
    args = parser.parse_args()
    if args.stride < 1:
        parser.error("--stride必须大于0")
    report = run(args)
    print(json.dumps({"effective_distance_mm": report["effective_distance_mm"],
                      "height_zero_mm": report["height_zero_mm"],
                      "candidate_frames": report["candidate_frames"],
                      "inlier_frames": report["inlier_frames"],
                      "rmse_px": report["height_model_point_metrics"]["rmse_px"],
                      "cv_rmse_px": report["cross_validation"]["point_metrics"]["rmse_px"],
                      "outputs": report["outputs"]},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

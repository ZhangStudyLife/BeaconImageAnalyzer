#!/usr/bin/env python3
"""Fit the down-camera 7 m closed ground boundary from an HCAL recording."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import warnings
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

warnings.filterwarnings("ignore", message="A NumPy version")
from scipy.optimize import least_squares


GROUND_RANGE_MM = 7000.0
MIN_HEIGHT_MM = 500.0
BOOTSTRAP_STRIDE = 5
CAR_THRESHOLD = 160
STRIP_THRESHOLD = 55
GUIDED_RADIUS_PX = 8.0
FRAME_INLIER_RMSE_PX = 2.0
RANSAC_ITERATIONS = 500
RANSAC_SEED = 0x444F574E
BOUNDARY_SAMPLES = 360
NORMALIZATION_SCALE = 93.5
BASE_BODY_TO_CAMERA = np.array(
    [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
PARAMETER_LOW = np.array([75.0, 45.0, 0.5, -2.0, -1.5,
                          -0.7, -0.7, -0.7, -500.0])
PARAMETER_HIGH = np.array([110.0, 80.0, 3.0, 2.0, 1.5,
                           0.7, 0.7, 0.7, 500.0])


@dataclass
class Metadata:
    frame_index: int
    sequence: int
    host_time_ms: int
    roll_deg: float
    pitch_deg: float
    height_mm: float
    attitude_valid: bool
    height_valid: bool


@dataclass
class Component:
    center: np.ndarray
    points: np.ndarray
    area: int
    mean: float
    length: float
    width: float
    boundary_index: int = -1
    distance: float = math.inf


@dataclass
class Candidate:
    metadata: Metadata
    points: np.ndarray
    quality: float
    azimuth_bin: int = -1


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def body_to_camera(parameters: np.ndarray) -> np.ndarray:
    return rotation_matrix(*parameters[5:8]) @ BASE_BODY_TO_CAMERA


def gravity(roll_deg, pitch_deg) -> np.ndarray:
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    return np.column_stack((-np.sin(pitch),
                            np.sin(roll) * np.cos(pitch),
                            np.cos(roll) * np.cos(pitch)))


def theta_for_radius(radius: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    radius2 = radius * radius
    return radius * (coefficients[0]
                     + radius2 * (coefficients[1] + radius2 * coefficients[2]))


def theta_derivative(radius: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    radius2 = radius * radius
    return coefficients[0] + radius2 * (
        3.0 * coefficients[1] + 5.0 * coefficients[2] * radius2)


def inverse_theta(theta: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    low = np.zeros_like(theta)
    high = np.full_like(theta, 2.0)
    for _ in range(36):
        middle = (low + high) * 0.5
        below = theta_for_radius(middle, coefficients) < theta
        low = np.where(below, middle, low)
        high = np.where(below, high, middle)
    return (low + high) * 0.5


def project_boundary(parameters: np.ndarray, metadata: Metadata) -> np.ndarray:
    roll = math.radians(metadata.roll_deg)
    pitch = math.radians(metadata.pitch_deg)
    g = np.array([-math.sin(pitch),
                  math.sin(roll) * math.cos(pitch),
                  math.cos(roll) * math.cos(pitch)], dtype=np.float64)
    first = np.cross(g, np.array([1.0, 0.0, 0.0]))
    if np.linalg.norm(first) < 1e-6:
        first = np.cross(g, np.array([0.0, 1.0, 0.0]))
    first /= np.linalg.norm(first)
    second = np.cross(g, first)
    angle = np.linspace(0.0, 2.0 * math.pi, BOUNDARY_SAMPLES, endpoint=False)
    height = metadata.height_mm + parameters[8]
    rays = (height * g[:, None]
            + GROUND_RANGE_MM
            * (np.cos(angle)[None, :] * first[:, None]
               + np.sin(angle)[None, :] * second[:, None]))
    rays /= np.linalg.norm(rays, axis=0)
    camera = body_to_camera(parameters) @ rays
    theta = np.arccos(np.clip(camera[2], -1.0, 1.0))
    radius = inverse_theta(theta, parameters[2:5])
    lateral = np.hypot(camera[0], camera[1])
    factor = np.divide(radius, lateral, out=np.zeros_like(radius), where=lateral > 1e-9)
    return np.column_stack((parameters[0] + NORMALIZATION_SCALE * camera[0] * factor,
                            parameters[1] + NORMALIZATION_SCALE * camera[1] * factor))


def distance_to_boundary(points: np.ndarray, boundary: np.ndarray) -> np.ndarray:
    first = boundary
    second = np.roll(boundary, -1, axis=0)
    direction = second - first
    length2 = np.sum(direction * direction, axis=1)
    result = np.full(len(points), math.inf, dtype=np.float64)
    for index, point in enumerate(points):
        fraction = np.divide(np.sum((point - first) * direction, axis=1), length2,
                             out=np.zeros_like(length2), where=length2 > 1e-12)
        fraction = np.clip(fraction, 0.0, 1.0)
        nearest = first + fraction[:, None] * direction
        result[index] = np.sqrt(np.min(np.sum((nearest - point) ** 2, axis=1)))
    return result


def frame_error(parameters: np.ndarray, candidate: Candidate) -> float:
    errors = distance_to_boundary(candidate.points,
                                  project_boundary(parameters, candidate.metadata))
    return float(np.sqrt(np.mean(errors * errors)))


def load_metadata(path: Path) -> list[Metadata]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    required = {"frame_index", "bimg_sequence", "host_time_ms", "roll_deg",
                "pitch_deg", "height_mm", "attitude_valid", "height_valid"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError("HCAL CSV does not contain attitude and height metadata")
    metadata = []
    for row in rows:
        item = Metadata(int(row["frame_index"]), int(row["bimg_sequence"]),
                        int(row["host_time_ms"]), float(row["roll_deg"]),
                        float(row["pitch_deg"]), float(row["height_mm"]),
                        row["attitude_valid"] == "1", row["height_valid"] == "1")
        if item.frame_index != len(metadata):
            raise ValueError("HCAL CSV frame indices are not continuous")
        metadata.append(item)
    return metadata


def load_prior(path: Path) -> np.ndarray:
    root = json.loads(path.read_text(encoding="utf-8"))
    if (root.get("format") != "horizon_model"
            or root.get("model_type") != "central_fisheye_angle_poly5"):
        raise ValueError("The prior must be a central_fisheye_angle_poly5 model")
    theta = np.asarray(root["theta_coefficients"], dtype=np.float64)
    if theta.shape != (3,):
        raise ValueError("The prior fisheye coefficients are invalid")
    return np.array([float(root["center_x"]), float(root["center_y"]),
                     theta[0], theta[1], theta[2], 0.0, 0.0, 0.0, 0.0])


def car_mask(gray: np.ndarray) -> np.ndarray:
    binary = (gray >= CAR_THRESHOLD).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    mask = np.zeros_like(binary)
    for index in range(1, count):
        if stats[index, cv2.CC_STAT_AREA] >= 20:
            mask[labels == index] = 1
    return cv2.dilate(mask, np.ones((9, 9), dtype=np.uint8))


def strip_components(gray: np.ndarray) -> list[Component]:
    binary = ((gray >= STRIP_THRESHOLD) & (car_mask(gray) == 0)).astype(np.uint8)
    count, labels, stats, centers = cv2.connectedComponentsWithStats(binary, 8)
    result = []
    for index in range(1, count):
        x, y, width, height, area = map(int, stats[index])
        if not 1 <= area <= 250 or max(width, height) > 45:
            continue
        pixels = np.column_stack(np.where(labels == index))[:, ::-1].astype(np.float64)
        (_, _), (rect_width, rect_height), angle = cv2.minAreaRect(pixels.astype(np.float32))
        length = max(rect_width, rect_height) + 1.0
        thickness = min(rect_width, rect_height) + 1.0
        ordinary = area <= 35 and max(width, height) <= 18
        thin = length >= 10.0 and thickness <= 4.5 and length / max(thickness, 1e-6) >= 2.8
        if not ordinary and not thin:
            continue
        center = np.asarray(centers[index], dtype=np.float64)
        annotation = center[None, :]
        if thin:
            radians = math.radians(angle + (90.0 if rect_width < rect_height else 0.0))
            direction = np.array([math.cos(radians), math.sin(radians)])
            half = direction * max(2.0, (length - 1.0) * 0.5)
            annotation = np.vstack((center - half, center + half))
        result.append(Component(center, annotation, area,
                                float(np.mean(gray[labels == index])), length, thickness))
    return result


def bootstrap_candidate(gray: np.ndarray, metadata: Metadata) -> Candidate | None:
    best = None
    for side in (0, 1):
        components = [item for item in strip_components(gray)
                      if (item.center[0] <= 38.0 if side == 0 else item.center[0] >= 149.0)]
        if len(components) < 4:
            continue
        generator = random.Random(RANSAC_SEED + metadata.frame_index + side)
        for _ in range(min(250, len(components) * len(components))):
            first, second = generator.sample(components, 2)
            if abs(first.center[1] - second.center[1]) < 8.0:
                continue
            slope = ((second.center[0] - first.center[0])
                     / (second.center[1] - first.center[1]))
            if abs(slope) > 1.2:
                continue
            intercept = first.center[0] - slope * first.center[1]
            inliers = [item for item in components
                       if abs(item.center[0] - (slope * item.center[1] + intercept)) <= 2.2]
            if len(inliers) < 4:
                continue
            span = max(item.center[1] for item in inliers) - min(item.center[1] for item in inliers)
            if span < 18.0:
                continue
            quality = span + 4.0 * len(inliers) + sum(
                item.area * item.mean / 100.0 for item in inliers)
            points = np.vstack([item.points for item in inliers])
            candidate = Candidate(metadata, points, quality)
            if best is None or candidate.quality > best.quality:
                best = candidate
    return best


def guided_candidate(gray: np.ndarray, metadata: Metadata,
                     parameters: np.ndarray) -> Candidate | None:
    boundary = project_boundary(parameters, metadata)
    components = []
    for item in strip_components(gray):
        distances2 = np.sum((boundary - item.center) ** 2, axis=1)
        item.boundary_index = int(np.argmin(distances2))
        item.distance = math.sqrt(float(distances2[item.boundary_index]))
        if item.distance <= GUIDED_RADIUS_PX:
            components.append(item)
    best = None
    for seed in components:
        group = [item for item in components
                 if min(abs(item.boundary_index - seed.boundary_index),
                        BOUNDARY_SAMPLES - abs(item.boundary_index - seed.boundary_index)) <= 40]
        if not group:
            continue
        centers = np.vstack([item.center for item in group])
        span = float(np.max(np.linalg.norm(centers - centers[0], axis=1)))
        area = sum(item.area for item in group)
        strong = any(item.length >= 12.0 and item.width <= 4.5 for item in group)
        if not ((((len(group) >= 3 or area >= 8) and span >= 4.0) or strong)):
            continue
        quality = (area + 4.0 * len(group) + span
                   - 2.0 * sum(item.distance for item in group))
        group.sort(key=lambda item: item.boundary_index)
        points = np.vstack([item.points for item in group])
        candidate = Candidate(metadata, points, quality)
        if best is None or candidate.quality > best.quality:
            best = candidate
    return best


def point_arrays(candidates: list[Candidate], selected: np.ndarray | None = None):
    if selected is None:
        selected = np.ones(len(candidates), dtype=bool)
    chosen = [item for index, item in enumerate(candidates) if selected[index]]
    points = np.vstack([item.points for item in chosen])
    roll = np.concatenate([np.full(len(item.points), item.metadata.roll_deg) for item in chosen])
    pitch = np.concatenate([np.full(len(item.points), item.metadata.pitch_deg) for item in chosen])
    height = np.concatenate([np.full(len(item.points), item.metadata.height_mm) for item in chosen])
    frame_ids = np.concatenate([np.full(len(item.points), index, dtype=np.int32)
                                for index, item in enumerate(chosen)])
    return points, roll, pitch, height, frame_ids


def angular_residual(parameters: np.ndarray, points: np.ndarray, roll: np.ndarray,
                     pitch: np.ndarray, height: np.ndarray) -> np.ndarray:
    dx = (points[:, 0] - parameters[0]) / NORMALIZATION_SCALE
    dy = (points[:, 1] - parameters[1]) / NORMALIZATION_SCALE
    radius = np.hypot(dx, dy)
    theta = theta_for_radius(radius, parameters[2:5])
    lateral = np.sin(theta)
    factor = np.divide(lateral, radius, out=np.ones_like(radius), where=radius > 1e-9)
    camera = np.column_stack((factor * dx, factor * dy, np.cos(theta)))
    body = camera @ body_to_camera(parameters)
    g = gravity(roll, pitch)
    vertical = np.sum(body * g, axis=1)
    horizontal = np.sqrt(np.maximum(0.0, 1.0 - np.clip(vertical, -1.0, 1.0) ** 2))
    observed = np.arctan2(horizontal, vertical)
    expected = np.arctan2(GROUND_RANGE_MM, height + parameters[8])
    return (observed - expected) * 80.0


def fit_parameters(candidates: list[Candidate], start: np.ndarray, prior: np.ndarray,
                   selected: np.ndarray | None = None, max_evaluations: int = 500) -> np.ndarray:
    points, roll, pitch, height, _ = point_arrays(candidates, selected)
    prior_scale = np.array([8.0, 8.0, 0.5, 0.8, 0.7,
                            0.35, 0.35, 0.35, 350.0])
    max_radius = math.hypot(max(start[0], 187.0 - start[0]),
                            max(start[1], 119.0 - start[1])) / NORMALIZATION_SCALE
    derivative_radius = np.linspace(0.0, max_radius, 17)

    def residual(parameters):
        derivative = theta_derivative(derivative_radius, parameters[2:5])
        monotonic = np.maximum(0.0, 0.05 - derivative) * 100.0
        return np.concatenate((angular_residual(parameters, points, roll, pitch, height),
                               (parameters - prior) / prior_scale, monotonic))

    result = least_squares(residual, np.clip(start, PARAMETER_LOW, PARAMETER_HIGH),
                           bounds=(PARAMETER_LOW, PARAMETER_HIGH), loss="huber",
                           f_scale=2.0, max_nfev=max_evaluations)
    return result.x


def fast_frame_errors(parameters: np.ndarray, candidates: list[Candidate]) -> np.ndarray:
    points, roll, pitch, height, frame_ids = point_arrays(candidates)
    errors = angular_residual(parameters, points, roll, pitch, height)
    result = np.zeros(len(candidates), dtype=np.float64)
    for index in range(len(candidates)):
        values = errors[frame_ids == index]
        result[index] = math.sqrt(float(np.mean(values * values)))
    return result


def ransac_seed(candidates: list[Candidate], start: np.ndarray) -> np.ndarray:
    generator = np.random.default_rng(RANSAC_SEED)
    jitter = np.array([1.5, 1.5, 0.08, 0.12, 0.08,
                       0.025, 0.025, 0.025, 50.0])
    best = start.copy()
    best_inliers = -1
    best_error = math.inf
    for iteration in range(RANSAC_ITERATIONS):
        hypothesis = start if iteration == 0 else np.clip(
            start + generator.normal(size=9) * jitter, PARAMETER_LOW, PARAMETER_HIGH)
        errors = fast_frame_errors(hypothesis, candidates)
        inliers = errors <= 2.5
        count = int(np.count_nonzero(inliers))
        squared = float(np.sum(errors[inliers] ** 2)) if count else math.inf
        if count > best_inliers or (count == best_inliers and squared < best_error):
            best, best_inliers, best_error = hypothesis, count, squared
    return best


def exact_errors(parameters: np.ndarray, candidates: list[Candidate]) -> np.ndarray:
    return np.asarray([frame_error(parameters, item) for item in candidates], dtype=np.float64)


def select_balanced(candidates: list[Candidate], parameters: np.ndarray) -> list[Candidate]:
    groups: dict[tuple[int, int], list[Candidate]] = {}
    for item in candidates:
        midpoint = np.mean(item.points, axis=0)
        angle = (math.atan2(midpoint[1] - parameters[1], midpoint[0] - parameters[0])
                 + 2.0 * math.pi) % (2.0 * math.pi)
        item.azimuth_bin = min(71, int(angle * 72.0 / (2.0 * math.pi)))
        height_band = 0 if item.metadata.height_mm < 1050.0 else (
            1 if item.metadata.height_mm < 1175.0 else 2)
        groups.setdefault((item.azimuth_bin, height_band), []).append(item)
    selected: dict[int, Candidate] = {}
    for values in groups.values():
        values.sort(key=lambda item: item.quality, reverse=True)
        for item in values[:6]:
            selected[item.metadata.frame_index] = item
    for key in (lambda item: item.metadata.roll_deg,
                lambda item: -item.metadata.roll_deg,
                lambda item: item.metadata.pitch_deg,
                lambda item: -item.metadata.pitch_deg,
                lambda item: item.metadata.height_mm,
                lambda item: -item.metadata.height_mm):
        for item in sorted(candidates, key=key, reverse=True)[:12]:
            selected[item.metadata.frame_index] = item
    return sorted(selected.values(), key=lambda item: item.metadata.frame_index)


def assign_azimuth_bins(candidates: list[Candidate], parameters: np.ndarray):
    for item in candidates:
        midpoint = np.mean(item.points, axis=0)
        angle = (math.atan2(midpoint[1] - parameters[1], midpoint[0] - parameters[0])
                 + 2.0 * math.pi) % (2.0 * math.pi)
        item.azimuth_bin = min(71, int(angle * 72.0 / (2.0 * math.pi)))


def add_rare_azimuth_samples(candidates: list[Candidate], selected: list[Candidate],
                             parameters: np.ndarray) -> list[Candidate]:
    assign_azimuth_bins(candidates, parameters)
    assign_azimuth_bins(selected, parameters)
    selected_errors = exact_errors(parameters, selected)
    covered = {item.azimuth_bin for index, item in enumerate(selected)
               if selected_errors[index] <= FRAME_INLIER_RMSE_PX}
    candidate_errors = exact_errors(parameters, candidates)
    selected_frames = {item.metadata.frame_index for item in selected}
    additions = []
    for azimuth_bin in range(72):
        if azimuth_bin in covered:
            continue
        choices = [(candidate_errors[index], -item.quality, item)
                   for index, item in enumerate(candidates)
                   if item.azimuth_bin == azimuth_bin
                   and item.metadata.frame_index not in selected_frames
                   and candidate_errors[index] <= FRAME_INLIER_RMSE_PX]
        choices.sort(key=lambda choice: (choice[0], choice[1]))
        additions.extend(choice[2] for choice in choices[:4])
    if not additions:
        return selected
    return sorted(selected + additions, key=lambda item: item.metadata.frame_index)


def refine_model(candidates: list[Candidate], start: np.ndarray, prior: np.ndarray):
    parameters = ransac_seed(candidates, start)
    selected = None
    for _ in range(4):
        parameters = fit_parameters(candidates, parameters, prior, selected)
        errors = exact_errors(parameters, candidates)
        classified = errors <= FRAME_INLIER_RMSE_PX
        if selected is not None and np.array_equal(selected, classified):
            break
        selected = classified
    return parameters, exact_errors(parameters, candidates)


def cross_validate(candidates: list[Candidate], final: np.ndarray,
                   prior: np.ndarray) -> dict:
    errors = []
    frame_limit = max(item.metadata.frame_index for item in candidates) + 1
    for fold in range(5):
        validation = np.asarray([
            min(4, item.metadata.frame_index * 5 // frame_limit) == fold
            for item in candidates], dtype=bool)
        if not np.any(validation) or np.count_nonzero(~validation) < 20:
            continue
        model = fit_parameters(candidates, final, prior, ~validation, 250)
        fold_errors = exact_errors(model, [item for index, item in enumerate(candidates)
                                           if validation[index]])
        errors.extend(fold_errors.tolist())
    values = np.asarray(errors, dtype=np.float64)
    return metrics(values)


def metrics(values: np.ndarray) -> dict:
    if not len(values):
        return {"count": 0, "rmse_px": None, "median_px": None,
                "p90_px": None, "max_px": None}
    return {"count": int(len(values)),
            "rmse_px": float(np.sqrt(np.mean(values * values))),
            "median_px": float(np.median(values)),
            "p90_px": float(np.percentile(values, 90.0)),
            "max_px": float(np.max(values))}


def matrix_values(parameters: np.ndarray) -> list[list[float]]:
    return body_to_camera(parameters).tolist()


def output_paths(prefix: Path) -> dict[str, Path]:
    base = str(prefix)
    return {"session": Path(base + "_down_auto.hcal.json"),
            "csv": Path(base + "_down_auto.hcal.csv"),
            "model": Path(base + "_down_ground_range_model.json"),
            "header": Path(base + "_down_ground_range_model.h"),
            "overlay": Path(base + "_down_ground_range_overlay.avi"),
            "report": Path(base + "_down_ground_range_report.json")}


def write_csv(path: Path, metadata: list[Metadata]):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["frame_index", "bimg_sequence", "host_time_ms", "camera_id",
                         "source_camera_id", "roll_deg", "pitch_deg", "height_mm",
                         "attitude_valid", "height_valid"])
        for item in metadata:
            writer.writerow([item.frame_index, item.sequence, item.host_time_ms, 2, 0,
                             f"{item.roll_deg:.9g}", f"{item.pitch_deg:.9g}",
                             f"{item.height_mm:.9g}", int(item.attitude_valid),
                             int(item.height_valid)])


def json_points(points: np.ndarray) -> list[dict]:
    if len(points) > 20:
        indices = np.linspace(0, len(points) - 1, 20).round().astype(int)
        points = points[indices]
    return [{"x": float(point[0]), "y": float(point[1])} for point in points]


def write_session(path: Path, source: dict, source_session: Path, source_video: Path,
                  csv_path: Path, candidates: list[Candidate], inliers: np.ndarray):
    annotations = []
    for index, item in enumerate(candidates):
        if inliers[index] and len(item.points) >= 2:
            annotations.append({"frame_index": item.metadata.frame_index,
                                "skipped": False,
                                "annotation_type": "curve_points",
                                "points": json_points(item.points)})
    root = dict(source)
    root.update({"version": 4, "camera_id": 2, "camera_name": "Down",
                 "source_camera_id": 0,
                 "video_file": str(Path(source_video).relative_to(path.parent)),
                 "csv_file": str(csv_path.relative_to(path.parent)),
                 "annotations": annotations, "fit": {},
                 "derived_from": str(source_session.resolve())})
    path.write_text(json.dumps(root, ensure_ascii=False, indent=4), encoding="utf-8")


def write_model(path: Path, parameters: np.ndarray, candidates: list[Candidate],
                errors: np.ndarray, inliers: np.ndarray, coverage: float, exportable: bool):
    accepted = [item for index, item in enumerate(candidates) if inliers[index]]
    values = errors[inliers]
    root = {"format": "horizon_model", "version": 1,
            "model_type": "down_ground_range_fisheye", "camera_id": 2,
            "camera_name": "Down", "image_width": 188, "image_height": 120,
            "coordinate_system": "Air FRD, degrees, millimeters, image pixels",
            "center_x": float(parameters[0]), "center_y": float(parameters[1]),
            "normalization_scale": NORMALIZATION_SCALE,
            "theta_coefficients": parameters[2:5].tolist(),
            "body_to_camera": matrix_values(parameters),
            "effective_range_mm": GROUND_RANGE_MM,
            "height_bias_mm": float(parameters[8]),
            "inlier_count": int(np.count_nonzero(inliers)),
            "sample_count": len(candidates),
            "rmse_px": float(np.sqrt(np.mean(values * values))),
            "median_error_px": float(np.median(values)),
            "max_error_px": float(np.max(values)),
            "roll_min_deg": min(item.metadata.roll_deg for item in accepted),
            "roll_max_deg": max(item.metadata.roll_deg for item in accepted),
            "pitch_min_deg": min(item.metadata.pitch_deg for item in accepted),
            "pitch_max_deg": max(item.metadata.pitch_deg for item in accepted),
            "height_min_mm": min(item.metadata.height_mm for item in accepted),
            "height_max_mm": max(item.metadata.height_mm for item in accepted),
            "azimuth_coverage_percent": coverage,
            "exportable": exportable,
            "extrinsic_rotation_rad": parameters[5:8].tolist()}
    path.write_text(json.dumps(root, ensure_ascii=False, indent=4), encoding="utf-8")


def float_literal(value: float) -> str:
    text = f"{value:.9g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text + "f"


def write_header(path: Path, parameters: np.ndarray):
    matrix = body_to_camera(parameters).reshape(-1)
    max_radius = math.hypot(max(parameters[0], 187.0 - parameters[0]),
                            max(parameters[1], 119.0 - parameters[1])) / NORMALIZATION_SCALE
    edge_theta = float(theta_for_radius(np.asarray(max_radius), parameters[2:5]))
    edge_slope = max(0.05, float(theta_derivative(np.asarray(max_radius), parameters[2:5])))
    theta = ", ".join(float_literal(value) for value in parameters[2:5])
    matrix_text = ", ".join(float_literal(value) for value in matrix)
    source = f"""#ifndef DOWN_GROUND_RANGE_MODEL_H
#define DOWN_GROUND_RANGE_MODEL_H

#include <math.h>

static const float down_ground_range_center_x = {float_literal(parameters[0])};
static const float down_ground_range_center_y = {float_literal(parameters[1])};
static const float down_ground_range_scale = {float_literal(NORMALIZATION_SCALE)};
static const float down_ground_range_theta[3] = {{{theta}}};
static const float down_ground_range_body_to_camera[9] = {{{matrix_text}}};
static const float down_ground_range_mm = {float_literal(GROUND_RANGE_MM)};
static const float down_ground_height_bias_mm = {float_literal(parameters[8])};
static const float down_ground_range_edge_radius = {float_literal(max_radius)};
static const float down_ground_range_edge_theta = {float_literal(edge_theta)};
static const float down_ground_range_edge_slope = {float_literal(edge_slope)};

static inline unsigned char down_ground_range_boundary(float roll_deg, float pitch_deg,
                                                        float height_mm, unsigned short index,
                                                        float *x, float *y)
{{
    const float pi = 3.14159265358979323846f;
    const float roll = roll_deg * pi / 180.0f, pitch = pitch_deg * pi / 180.0f;
    float g[3] = {{-sinf(pitch), sinf(roll) * cosf(pitch), cosf(roll) * cosf(pitch)}};
    float u[3] = {{0.0f, g[2], -g[1]}}, un = sqrtf(u[1] * u[1] + u[2] * u[2]);
    if (un < 1e-6f) {{ u[0] = -g[2]; u[1] = 0.0f; u[2] = g[0];
        un = sqrtf(u[0] * u[0] + u[2] * u[2]); }}
    u[0] /= un; u[1] /= un; u[2] /= un;
    float v[3] = {{g[1] * u[2] - g[2] * u[1], g[2] * u[0] - g[0] * u[2],
                   g[0] * u[1] - g[1] * u[0]}};
    const float a = 2.0f * pi * (float)(index % 360U) / 360.0f;
    const float h = height_mm + down_ground_height_bias_mm;
    float d[3] = {{h * g[0] + down_ground_range_mm * (cosf(a) * u[0] + sinf(a) * v[0]),
                   h * g[1] + down_ground_range_mm * (cosf(a) * u[1] + sinf(a) * v[1]),
                   h * g[2] + down_ground_range_mm * (cosf(a) * u[2] + sinf(a) * v[2])}};
    float n = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), c[3];
    if (n <= 1e-6f || h <= 0.0f) return 0U;
    d[0] /= n; d[1] /= n; d[2] /= n;
    c[0] = down_ground_range_body_to_camera[0] * d[0] + down_ground_range_body_to_camera[1] * d[1] + down_ground_range_body_to_camera[2] * d[2];
    c[1] = down_ground_range_body_to_camera[3] * d[0] + down_ground_range_body_to_camera[4] * d[1] + down_ground_range_body_to_camera[5] * d[2];
    c[2] = down_ground_range_body_to_camera[6] * d[0] + down_ground_range_body_to_camera[7] * d[1] + down_ground_range_body_to_camera[8] * d[2];
    const float theta = acosf(fmaxf(-1.0f, fminf(1.0f, c[2])));
    const float lateral = sqrtf(c[0] * c[0] + c[1] * c[1]);
    float lo = 0.0f, hi = down_ground_range_edge_radius, radius;
    if (theta > down_ground_range_edge_theta)
        radius = down_ground_range_edge_radius + (theta - down_ground_range_edge_theta) / down_ground_range_edge_slope;
    else {{
        unsigned char k;
        for (k = 0U; k < 28U; ++k) {{
            float r = (lo + hi) * 0.5f, r2 = r * r;
            float t = r * (down_ground_range_theta[0] + r2 * (down_ground_range_theta[1] + r2 * down_ground_range_theta[2]));
            if (t < theta) lo = r; else hi = r;
        }}
        radius = (lo + hi) * 0.5f;
    }}
    if (lateral <= 1e-6f) {{ *x = down_ground_range_center_x; *y = down_ground_range_center_y; return 1U; }}
    {{ const float r = radius / lateral;
       *x = down_ground_range_center_x + down_ground_range_scale * c[0] * r;
       *y = down_ground_range_center_y + down_ground_range_scale * c[1] * r; }}
    return 1U;
}}

#endif
"""
    path.write_text(source, encoding="ascii")


def draw_boundary(frame: np.ndarray, boundary: np.ndarray, color):
    for index in range(len(boundary)):
        first = boundary[index]
        second = boundary[(index + 1) % len(boundary)]
        if (0 <= first[0] < frame.shape[1] and 0 <= first[1] < frame.shape[0]
                and 0 <= second[0] < frame.shape[1] and 0 <= second[1] < frame.shape[0]):
            cv2.line(frame, tuple(np.rint(first).astype(int)),
                     tuple(np.rint(second).astype(int)), color, 1, cv2.LINE_AA)


def write_overlay(path: Path, video_path: Path, metadata: list[Metadata],
                  parameters: np.ndarray, candidates: list[Candidate], inliers: np.ndarray):
    capture = cv2.VideoCapture(str(video_path))
    fps = capture.get(cv2.CAP_PROP_FPS) or 50.0
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (188, 120))
    annotations = {item.metadata.frame_index: (item, bool(inliers[index]))
                   for index, item in enumerate(candidates)}
    for item in metadata:
        ok, frame = capture.read()
        if not ok:
            break
        draw_boundary(frame, project_boundary(parameters, item), (0, 255, 0))
        annotation = annotations.get(item.frame_index)
        if annotation is not None:
            candidate, accepted = annotation
            color = (0, 255, 255) if accepted else (0, 0, 255)
            for point in candidate.points:
                cv2.circle(frame, tuple(np.rint(point).astype(int)), 1, color, -1)
        writer.write(frame)
    capture.release()
    writer.release()


def run(args) -> dict:
    session_path = args.session.resolve()
    source = json.loads(session_path.read_text(encoding="utf-8"))
    if (source.get("format") != "horizon_calibration"
            or not source.get("height_recorded") or int(source.get("frame_count", 0)) <= 0):
        raise ValueError("The source must be a complete height-enabled HCAL session")
    video_path = (session_path.parent / source["video_file"]).resolve()
    csv_path = (session_path.parent / source["csv_file"]).resolve()
    metadata = load_metadata(csv_path)
    if len(metadata) != int(source["frame_count"]):
        raise ValueError("HCAL frame count does not match the CSV")
    prior = load_prior(args.prior_model.resolve())

    capture = cv2.VideoCapture(str(video_path))
    bootstrap = []
    for item in metadata[::BOOTSTRAP_STRIDE]:
        if (not item.attitude_valid or not item.height_valid or item.height_mm < 800.0
                or abs(item.roll_deg) > 18.0 or abs(item.pitch_deg) > 18.0):
            continue
        capture.set(cv2.CAP_PROP_POS_FRAMES, item.frame_index)
        ok, frame = capture.read()
        if not ok:
            continue
        candidate = bootstrap_candidate(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), item)
        if candidate is not None and frame_error(prior, candidate) <= 10.0:
            bootstrap.append(candidate)
    if len(bootstrap) < 12:
        raise RuntimeError(f"Only {len(bootstrap)} bootstrap frames were extracted")
    initial = fit_parameters(bootstrap, prior, prior)
    bootstrap_errors = exact_errors(initial, bootstrap)
    bootstrap_inliers = bootstrap_errors <= 2.5
    if np.count_nonzero(bootstrap_inliers) >= 12:
        initial = fit_parameters(bootstrap, initial, prior, bootstrap_inliers)

    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
    extracted = []
    for item in metadata:
        ok, frame = capture.read()
        if not ok:
            break
        if (not item.attitude_valid or not item.height_valid
                or item.height_mm < MIN_HEIGHT_MM or float(np.mean(frame)) < 0.2):
            continue
        candidate = guided_candidate(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), item, initial)
        if candidate is not None:
            extracted.append(candidate)
    capture.release()
    if len(extracted) < 20:
        raise RuntimeError(f"Only {len(extracted)} guided frames were extracted")
    selected = select_balanced(extracted, initial)
    parameters, errors = refine_model(selected, initial, prior)
    augmented = add_rare_azimuth_samples(extracted, selected, parameters)
    if len(augmented) != len(selected):
        selected = augmented
        parameters, errors = refine_model(selected, parameters, prior)
    inliers = errors <= FRAME_INLIER_RMSE_PX
    if np.count_nonzero(inliers) < 20:
        raise RuntimeError("The fitted model has fewer than 20 inlier frames")

    assign_azimuth_bins(selected, parameters)
    bins = {item.azimuth_bin for index, item in enumerate(selected) if inliers[index]}
    coverage = len(bins) * 100.0 / 72.0
    accepted = [item for index, item in enumerate(selected) if inliers[index]]
    height_span = (max(item.metadata.height_mm for item in accepted)
                   - min(item.metadata.height_mm for item in accepted))
    inlier_metrics = metrics(errors[inliers])
    cv_metrics = cross_validate(selected, parameters, prior)
    exportable = (np.count_nonzero(inliers) >= 20 and coverage >= 75.0
                  and height_span >= 300.0 and inlier_metrics["rmse_px"] <= 1.5
                  and cv_metrics["rmse_px"] is not None and cv_metrics["rmse_px"] <= 2.0)

    outputs = output_paths(args.output_prefix.resolve())
    for path in outputs.values():
        path.parent.mkdir(parents=True, exist_ok=True)
    write_csv(outputs["csv"], metadata)
    write_session(outputs["session"], source, session_path, video_path,
                  outputs["csv"], selected, inliers)
    write_model(outputs["model"], parameters, selected, errors, inliers,
                coverage, exportable)
    write_header(outputs["header"], parameters)
    write_overlay(outputs["overlay"], video_path, metadata, parameters, selected, inliers)

    report = {"source_session": str(session_path), "source_video": str(video_path),
              "source_csv": str(csv_path), "decoded_frames": len(metadata),
              "bootstrap_frames": len(bootstrap), "guided_frames": len(extracted),
              "selected_frames": len(selected),
              "inlier_frames": int(np.count_nonzero(inliers)),
              "outlier_frames": int(len(selected) - np.count_nonzero(inliers)),
              "azimuth_coverage_percent": coverage,
              "height_span_mm": height_span, "effective_range_mm": GROUND_RANGE_MM,
              "parameters": {"center_x": float(parameters[0]),
                             "center_y": float(parameters[1]),
                             "theta_coefficients": parameters[2:5].tolist(),
                             "extrinsic_rotation_rad": parameters[5:8].tolist(),
                             "height_bias_mm": float(parameters[8])},
              "inlier_frame_metrics": inlier_metrics,
              "cross_validation_frame_metrics": cv_metrics,
              "exportable": exportable,
              "extraction": {"minimum_height_mm": MIN_HEIGHT_MM,
                             "car_threshold": CAR_THRESHOLD,
                             "strip_threshold": STRIP_THRESHOLD,
                             "guided_radius_px": GUIDED_RADIUS_PX,
                             "frame_inlier_rmse_px": FRAME_INLIER_RMSE_PX,
                             "ransac_iterations": RANSAC_ITERATIONS},
              "outputs": {name: str(path.resolve()) for name, path in outputs.items()}}
    outputs["report"].write_text(json.dumps(report, ensure_ascii=False, indent=4),
                                 encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--prior-model", required=True, type=Path)
    parser.add_argument("--output-prefix", required=True, type=Path)
    args = parser.parse_args()
    report = run(args)
    print(json.dumps({"guided_frames": report["guided_frames"],
                      "selected_frames": report["selected_frames"],
                      "inlier_frames": report["inlier_frames"],
                      "azimuth_coverage_percent": report["azimuth_coverage_percent"],
                      "rmse_px": report["inlier_frame_metrics"]["rmse_px"],
                      "cv_rmse_px": report["cross_validation_frame_metrics"]["rmse_px"],
                      "exportable": report["exportable"],
                      "outputs": report["outputs"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

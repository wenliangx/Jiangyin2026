"""Square-board localization and multi-template classification."""

from dataclasses import dataclass, field
import math
import os
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np


LABELS = ("plane", "car", "ship", "house")


@dataclass(frozen=True)
class MatcherConfig:
    canonical_size: int = 256
    border_crop_fraction: float = 0.04
    canny_low: int = 60
    canny_high: int = 160
    min_area_ratio: float = 0.005
    max_area_ratio: float = 0.80
    polygon_epsilon_ratio: float = 0.03
    min_side_ratio: float = 0.60
    max_angle_cosine: float = 0.35
    max_candidates: int = 5
    white_board_enabled: bool = True
    white_max_saturation: int = 60
    white_min_value: int = 170
    white_open_kernel: int = 5
    white_close_kernel: int = 5
    white_min_fill_ratio: float = 0.65
    white_max_area_ratio: float = 0.20
    min_target_side_px: float = 80.0
    min_sharpness: float = 20.0
    gray_weight: float = 0.50
    hog_weight: float = 0.30
    color_weight: float = 0.20
    min_class_score: float = 0.60
    min_class_margin: float = 0.03
    min_candidate_margin: float = 0.02
    orb_nfeatures: int = 1200
    orb_max_width: int = 960
    orb_ratio_test: float = 0.75
    orb_ransac_reproj_px: float = 5.0
    orb_min_good_matches: int = 12
    orb_min_inliers: int = 10
    orb_min_inlier_ratio: float = 0.45
    orb_min_inlier_margin: int = 5
    akaze_max_width: int = 960
    akaze_ratio_test: float = 0.80
    akaze_ransac_reproj_px: float = 5.0
    akaze_min_good_matches: int = 10
    akaze_min_inliers: int = 8
    akaze_min_inlier_ratio: float = 0.45
    akaze_min_inlier_margin: int = 4
    augmentations: Sequence[Mapping[str, Any]] = field(
        default_factory=lambda: ({},)
    )

    def __post_init__(self):
        if self.canonical_size <= 0 or self.canonical_size % 32:
            raise ValueError("canonical_size must be positive and divisible by 32")
        if not 0.0 <= self.border_crop_fraction < 0.25:
            raise ValueError("border_crop_fraction must be in [0, 0.25)")
        if not 0.0 < self.min_area_ratio < self.max_area_ratio <= 1.0:
            raise ValueError("area ratios must satisfy 0 < min < max <= 1")
        if self.max_candidates <= 0:
            raise ValueError("max_candidates must be positive")
        if not 0 <= self.white_max_saturation <= 255:
            raise ValueError("white_max_saturation must be in [0, 255]")
        if not 0 <= self.white_min_value <= 255:
            raise ValueError("white_min_value must be in [0, 255]")
        for name, size in (
            ("white_open_kernel", self.white_open_kernel),
            ("white_close_kernel", self.white_close_kernel),
        ):
            if size <= 0 or size % 2 == 0:
                raise ValueError(f"{name} must be a positive odd integer")
        if not 0.0 < self.white_min_fill_ratio <= 1.0:
            raise ValueError("white_min_fill_ratio must be in (0, 1]")
        if not self.min_area_ratio <= self.white_max_area_ratio <= self.max_area_ratio:
            raise ValueError(
                "white_max_area_ratio must be within the global area limits"
            )
        if self.orb_nfeatures <= 0 or self.orb_max_width <= 0:
            raise ValueError("ORB feature count and max width must be positive")
        if not 0.0 < self.orb_ratio_test < 1.0:
            raise ValueError("orb_ratio_test must be in (0, 1)")
        if self.orb_ransac_reproj_px <= 0.0:
            raise ValueError("orb_ransac_reproj_px must be positive")
        if self.orb_min_good_matches < 4 or self.orb_min_inliers < 4:
            raise ValueError("ORB match thresholds must be at least four")
        if not 0.0 < self.orb_min_inlier_ratio <= 1.0:
            raise ValueError("orb_min_inlier_ratio must be in (0, 1]")
        if self.orb_min_inlier_margin < 0:
            raise ValueError("orb_min_inlier_margin must be non-negative")
        if self.akaze_max_width <= 0:
            raise ValueError("akaze_max_width must be positive")
        if not 0.0 < self.akaze_ratio_test < 1.0:
            raise ValueError("akaze_ratio_test must be in (0, 1)")
        if self.akaze_ransac_reproj_px <= 0.0:
            raise ValueError("akaze_ransac_reproj_px must be positive")
        if self.akaze_min_good_matches < 4 or self.akaze_min_inliers < 4:
            raise ValueError("AKAZE match thresholds must be at least four")
        if not 0.0 < self.akaze_min_inlier_ratio <= 1.0:
            raise ValueError("akaze_min_inlier_ratio must be in (0, 1]")
        if self.akaze_min_inlier_margin < 0:
            raise ValueError("akaze_min_inlier_margin must be non-negative")
        weight_sum = self.gray_weight + self.hog_weight + self.color_weight
        if not math.isclose(weight_sum, 1.0, abs_tol=1e-6):
            raise ValueError("gray, HOG, and color weights must sum to 1")
        if not self.augmentations:
            raise ValueError("at least one augmentation tuple is required")

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]):
        allowed = {item.name for item in cls.__dataclass_fields__.values()}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"unknown matcher settings: {sorted(unknown)}")
        copied = dict(values)
        if "augmentations" in copied:
            copied["augmentations"] = tuple(copied["augmentations"])
        return cls(**copied)


@dataclass
class MatchResult:
    valid: bool = False
    label: str = "unknown"
    score: float = 0.0
    second_score: float = 0.0
    gray_score: float = 0.0
    hog_score: float = 0.0
    color_score: float = 0.0
    margin: float = 0.0
    sharpness: float = 0.0
    target_side_px: float = 0.0
    corners: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32)
    )
    candidate_margin: float = 0.0
    reason: str = ""


@dataclass
class _TemplateFeatures:
    gray: np.ndarray
    hog: np.ndarray
    histogram: np.ndarray


def order_quad(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).reshape(-1)
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(differences)]
    ordered[3] = points[np.argmax(differences)]
    return ordered


def measure_sharpness(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F, ksize=3).var())


class TargetMatcher:
    def __init__(self, config: MatcherConfig, templates_dir: str):
        self.config = config
        size = config.canonical_size
        self._hog = cv2.HOGDescriptor(
            (size, size),
            (32, 32),
            (16, 16),
            (16, 16),
            9,
        )
        self._templates = self._load_templates(templates_dir)
        self._orb = cv2.ORB_create(nfeatures=config.orb_nfeatures)
        self._orb_templates = self._load_orb_templates(templates_dir)
        self._akaze = cv2.AKAZE_create()
        self._akaze_templates = self._load_akaze_templates(templates_dir)

    def _load_templates(self, templates_dir: str):
        loaded: Dict[str, List[_TemplateFeatures]] = {}
        for label in LABELS:
            path = os.path.join(templates_dir, f"{label}.png")
            base = cv2.imread(path, cv2.IMREAD_COLOR)
            if base is None:
                raise FileNotFoundError(f"template not found or unreadable: {path}")
            variants = []
            for augmentation in self.config.augmentations:
                augmented = self._augment(base, augmentation)
                variants.append(self._extract_features(augmented))
            loaded[label] = variants
        return loaded

    def _load_orb_templates(self, templates_dir: str):
        loaded = {}
        for label in LABELS:
            path = os.path.join(templates_dir, f"{label}.png")
            image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise FileNotFoundError(f"template not found or unreadable: {path}")
            keypoints, descriptors = self._orb.detectAndCompute(image, None)
            if descriptors is None or len(keypoints) < 4:
                raise RuntimeError(f"template has too few ORB features: {path}")
            loaded[label] = (image.shape, keypoints, descriptors)
        return loaded

    def _load_akaze_templates(self, templates_dir: str):
        loaded = {}
        for label in LABELS:
            path = os.path.join(templates_dir, f"{label}.png")
            image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise FileNotFoundError(f"template not found or unreadable: {path}")
            keypoints, descriptors = self._akaze.detectAndCompute(image, None)
            if descriptors is None or len(keypoints) < 4:
                raise RuntimeError(f"template has too few AKAZE features: {path}")
            loaded[label] = (image.shape, keypoints, descriptors)
        return loaded

    def _augment(self, image: np.ndarray, values: Mapping[str, Any]):
        result = image.astype(np.float32)
        contrast = float(values.get("contrast", 1.0))
        brightness = float(values.get("brightness", 0.0))
        result = np.clip(result * contrast + brightness, 0, 255)

        color_shift = values.get("color_shift", (0.0, 0.0, 0.0))
        if len(color_shift) != 3:
            raise ValueError("color_shift must contain B, G, and R offsets")
        result += np.asarray(color_shift, dtype=np.float32).reshape(1, 1, 3)
        result = np.clip(result, 0, 255).astype(np.uint8)

        gamma = float(values.get("gamma", 1.0))
        if gamma <= 0.0:
            raise ValueError("gamma must be positive")
        if not math.isclose(gamma, 1.0):
            lookup = np.array(
                [((index / 255.0) ** gamma) * 255.0 for index in range(256)],
                dtype=np.uint8,
            )
            result = cv2.LUT(result, lookup)

        angle = float(values.get("angle", 0.0))
        scale = float(values.get("scale", 1.0))
        if scale <= 0.0:
            raise ValueError("scale must be positive")
        if angle or not math.isclose(scale, 1.0):
            height, width = result.shape[:2]
            matrix = cv2.getRotationMatrix2D(
                (width / 2.0, height / 2.0), angle, scale
            )
            result = cv2.warpAffine(
                result,
                matrix,
                (width, height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REFLECT_101,
            )

        blur = int(values.get("blur", 0))
        if blur:
            if blur < 0 or blur % 2 == 0:
                raise ValueError("blur must be zero or a positive odd integer")
            result = cv2.GaussianBlur(result, (blur, blur), 0)
        return result

    def _preprocess_patch(self, image: np.ndarray) -> np.ndarray:
        if image is None or image.size == 0:
            raise ValueError("patch is empty")
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("patch must be a BGR or grayscale image")

        height, width = image.shape[:2]
        crop = int(round(min(height, width) * self.config.border_crop_fraction))
        if crop:
            if height <= crop * 2 or width <= crop * 2:
                raise ValueError("border crop removes the entire patch")
            image = image[crop : height - crop, crop : width - crop]
        size = self.config.canonical_size
        return cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)

    def _extract_features(self, patch: np.ndarray) -> _TemplateFeatures:
        prepared = self._preprocess_patch(patch)
        gray = cv2.cvtColor(prepared, cv2.COLOR_BGR2GRAY)
        hog = self._hog.compute(gray).reshape(-1).astype(np.float32)
        hsv = cv2.cvtColor(prepared, cv2.COLOR_BGR2HSV)
        histogram = cv2.calcHist([hsv], [0, 1], None, [32, 32], [0, 180, 0, 256])
        histogram = cv2.normalize(
            histogram, None, alpha=1.0, norm_type=cv2.NORM_L1
        )
        return _TemplateFeatures(gray=gray, hog=hog, histogram=histogram)

    @staticmethod
    def _mapped_correlation(value: float) -> float:
        if not np.isfinite(value):
            return 0.0
        return float(np.clip((value + 1.0) * 0.5, 0.0, 1.0))

    def _score_features(
        self, candidate: _TemplateFeatures, template: _TemplateFeatures
    ) -> Tuple[float, float, float, float]:
        gray_raw = cv2.matchTemplate(
            candidate.gray, template.gray, cv2.TM_CCOEFF_NORMED
        )[0, 0]
        gray_score = self._mapped_correlation(float(gray_raw))

        denominator = float(
            np.linalg.norm(candidate.hog) * np.linalg.norm(template.hog)
        )
        hog_score = (
            float(np.dot(candidate.hog, template.hog) / denominator)
            if denominator > 1e-12
            else 0.0
        )
        hog_score = float(np.clip(hog_score, 0.0, 1.0))

        color_raw = cv2.compareHist(
            candidate.histogram, template.histogram, cv2.HISTCMP_CORREL
        )
        color_score = self._mapped_correlation(float(color_raw))

        total = (
            self.config.gray_weight * gray_score
            + self.config.hog_weight * hog_score
            + self.config.color_weight * color_score
        )
        return float(total), gray_score, hog_score, color_score

    def classify_patch(self, patch: np.ndarray) -> MatchResult:
        try:
            candidate = self._extract_features(patch)
        except (ValueError, cv2.error) as error:
            return MatchResult(reason=f"invalid_patch:{error}")

        sharpness = measure_sharpness(candidate.gray)
        class_scores = []
        for label in LABELS:
            best = None
            for template in self._templates[label]:
                scored = self._score_features(candidate, template)
                if best is None or scored[0] > best[0]:
                    best = scored
            class_scores.append((best[0], label, best[1], best[2], best[3]))

        class_scores.sort(key=lambda item: item[0], reverse=True)
        best, second = class_scores[0], class_scores[1]
        margin = float(best[0] - second[0])
        result = MatchResult(
            label=best[1],
            score=float(best[0]),
            second_score=float(second[0]),
            gray_score=float(best[2]),
            hog_score=float(best[3]),
            color_score=float(best[4]),
            margin=margin,
            sharpness=sharpness,
        )
        if sharpness < self.config.min_sharpness:
            result.label = "unknown"
            result.reason = "too_blurry"
            return result
        if result.score < self.config.min_class_score:
            result.label = "unknown"
            result.reason = "class_score_too_low"
            return result
        if result.margin < self.config.min_class_margin:
            result.label = "unknown"
            result.reason = "class_margin_too_low"
            return result
        result.valid = True
        result.reason = "accepted"
        return result

    @staticmethod
    def _angle_cosine(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        first = a - b
        second = c - b
        denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
        if denominator <= 1e-12:
            return 1.0
        return abs(float(np.dot(first, second) / denominator))

    def _quad_quality(
        self, quad: np.ndarray, frame_area: float
    ) -> Optional[Tuple[float, float]]:
        ordered = order_quad(quad)
        area = abs(float(cv2.contourArea(ordered)))
        area_ratio = area / frame_area
        if not self.config.min_area_ratio <= area_ratio <= self.config.max_area_ratio:
            return None
        sides = np.linalg.norm(np.roll(ordered, -1, axis=0) - ordered, axis=1)
        maximum = float(np.max(sides))
        minimum = float(np.min(sides))
        if maximum <= 1e-6 or minimum / maximum < self.config.min_side_ratio:
            return None
        angle_cosines = [
            self._angle_cosine(
                ordered[(index - 1) % 4],
                ordered[index],
                ordered[(index + 1) % 4],
            )
            for index in range(4)
        ]
        max_cosine = max(angle_cosines)
        if max_cosine > self.config.max_angle_cosine:
            return None
        rectangularity = 1.0 - max_cosine
        side_balance = minimum / maximum
        quality = area_ratio * rectangularity * side_balance
        return quality, float(np.mean(sides))

    def find_square_candidates(self, frame: np.ndarray):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(
            blurred, self.config.canny_low, self.config.canny_high
        )
        contours = cv2.findContours(
            edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )[0]
        frame_area = float(frame.shape[0] * frame.shape[1])
        ranked = []
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0.0:
                continue
            polygon = cv2.approxPolyDP(
                contour,
                self.config.polygon_epsilon_ratio * perimeter,
                True,
            )
            if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                continue
            ordered = order_quad(polygon.reshape(4, 2))
            quality = self._quad_quality(ordered, frame_area)
            if quality is None:
                continue
            ranked.append((quality[0], quality[1], ordered))

        ranked.extend(self._find_white_board_candidates(frame, frame_area))

        ranked.sort(key=lambda item: item[0], reverse=True)
        deduplicated = []
        for item in ranked:
            center = item[2].mean(axis=0)
            duplicate = False
            for kept in deduplicated:
                kept_center = kept[2].mean(axis=0)
                center_distance = float(np.linalg.norm(center - kept_center))
                if center_distance < 0.08 * max(item[1], kept[1]):
                    duplicate = True
                    break
            if not duplicate:
                deduplicated.append(item)
            if len(deduplicated) >= self.config.max_candidates:
                break
        return deduplicated

    def _find_white_board_candidates(
        self, frame: np.ndarray, frame_area: float
    ):
        """Locate bright, low-saturation competition boards.

        The edge-only contour path can merge a white board with mortar lines on
        the red brick-pattern walls used at the competition. Opening the HSV
        mask removes those thin lines before a minimum-area rectangle recovers
        the board boundary. Template classification and temporal voting remain
        responsible for rejecting unrelated white regions.
        """
        if not self.config.white_board_enabled:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array((0, 0, self.config.white_min_value), dtype=np.uint8),
            np.array(
                (179, self.config.white_max_saturation, 255),
                dtype=np.uint8,
            ),
        )
        for operation, size in (
            (cv2.MORPH_OPEN, self.config.white_open_kernel),
            (cv2.MORPH_CLOSE, self.config.white_close_kernel),
        ):
            if size > 1:
                kernel = cv2.getStructuringElement(
                    cv2.MORPH_RECT, (size, size)
                )
                mask = cv2.morphologyEx(mask, operation, kernel)

        ranked = []
        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[0]
        for contour in contours:
            contour_area = abs(float(cv2.contourArea(contour)))
            if contour_area <= 0.0:
                continue
            rectangle = cv2.minAreaRect(contour)
            width, height = rectangle[1]
            if min(width, height) <= 1.0:
                continue
            rectangle_area = float(width * height)
            area_ratio = rectangle_area / frame_area
            if area_ratio > self.config.white_max_area_ratio:
                continue
            fill_ratio = contour_area / rectangle_area
            if fill_ratio < self.config.white_min_fill_ratio:
                continue
            corners = order_quad(cv2.boxPoints(rectangle))
            quality = self._quad_quality(corners, frame_area)
            if quality is None:
                continue
            ranked.append(
                (quality[0] * fill_ratio, quality[1], corners)
            )
        return ranked

    def _warp_candidate(self, frame: np.ndarray, corners: np.ndarray):
        size = self.config.canonical_size
        destination = np.float32(
            [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]]
        )
        transform = cv2.getPerspectiveTransform(corners, destination)
        return cv2.warpPerspective(frame, transform, (size, size))

    def _match_orb_fallback(self, frame: np.ndarray) -> MatchResult:
        height, width = frame.shape[:2]
        scale_back = 1.0
        working = frame
        if width > self.config.orb_max_width:
            working_width = self.config.orb_max_width
            working_height = max(1, int(round(height * working_width / width)))
            working = cv2.resize(
                frame,
                (working_width, working_height),
                interpolation=cv2.INTER_AREA,
            )
            scale_back = float(width) / working_width

        gray = cv2.cvtColor(working, cv2.COLOR_BGR2GRAY)
        scene_keypoints, scene_descriptors = self._orb.detectAndCompute(
            gray, None
        )
        if scene_descriptors is None or len(scene_keypoints) < 4:
            return MatchResult(reason="orb_no_scene_features")

        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        frame_area = float(height * width)
        ranked = []
        for label in LABELS:
            template_shape, template_keypoints, template_descriptors = (
                self._orb_templates[label]
            )
            pairs = matcher.knnMatch(
                template_descriptors, scene_descriptors, k=2
            )
            good = [
                first
                for pair in pairs
                if len(pair) == 2
                for first, second in [pair]
                if first.distance
                < self.config.orb_ratio_test * second.distance
            ]
            if len(good) < 4:
                continue

            source = np.float32(
                [template_keypoints[item.queryIdx].pt for item in good]
            ).reshape(-1, 1, 2)
            destination = np.float32(
                [scene_keypoints[item.trainIdx].pt for item in good]
            ).reshape(-1, 1, 2)
            transform, mask = cv2.findHomography(
                source,
                destination,
                cv2.RANSAC,
                self.config.orb_ransac_reproj_px,
            )
            if transform is None or mask is None:
                continue

            template_height, template_width = template_shape
            template_corners = np.float32(
                [[
                    [0, 0],
                    [template_width - 1, 0],
                    [template_width - 1, template_height - 1],
                    [0, template_height - 1],
                ]]
            )
            corners = cv2.perspectiveTransform(
                template_corners, transform
            ).reshape(4, 2)
            corners *= scale_back
            quality = self._quad_quality(corners, frame_area)
            if quality is None:
                continue

            inliers = int(mask.sum())
            inlier_ratio = float(inliers) / len(good)
            ranked.append(
                (
                    inliers,
                    len(good),
                    inlier_ratio,
                    label,
                    quality[1],
                    order_quad(corners),
                )
            )

        if not ranked:
            return MatchResult(reason="orb_no_geometric_match")
        ranked.sort(key=lambda item: (item[0], item[2]), reverse=True)
        best = ranked[0]
        second_inliers = ranked[1][0] if len(ranked) > 1 else 0
        if best[1] < self.config.orb_min_good_matches:
            return MatchResult(reason="orb_too_few_matches")
        if best[0] < self.config.orb_min_inliers:
            return MatchResult(reason="orb_too_few_inliers")
        if best[2] < self.config.orb_min_inlier_ratio:
            return MatchResult(reason="orb_inlier_ratio_too_low")
        if best[0] - second_inliers < self.config.orb_min_inlier_margin:
            return MatchResult(reason="orb_inlier_margin_too_low")

        patch = self._warp_candidate(frame, best[5])
        result = self.classify_patch(patch)
        result.target_side_px = best[4]
        result.corners = best[5]
        if result.valid and result.label != best[3]:
            result.valid = False
            result.label = "unknown"
            result.reason = "orb_label_disagreement"
        elif result.valid:
            result.reason = "accepted_orb"
        return result

    def _match_akaze_fallback(self, frame: np.ndarray) -> MatchResult:
        height, width = frame.shape[:2]
        scale_back = 1.0
        working = frame
        if width > self.config.akaze_max_width:
            working_width = self.config.akaze_max_width
            working_height = max(1, int(round(height * working_width / width)))
            working = cv2.resize(
                frame,
                (working_width, working_height),
                interpolation=cv2.INTER_AREA,
            )
            scale_back = float(width) / working_width

        gray = cv2.cvtColor(working, cv2.COLOR_BGR2GRAY)
        scene_keypoints, scene_descriptors = self._akaze.detectAndCompute(
            gray, None
        )
        if scene_descriptors is None or len(scene_keypoints) < 4:
            return MatchResult(reason="akaze_no_scene_features")

        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        frame_area = float(height * width)
        ranked = []
        for label in LABELS:
            template_shape, template_keypoints, template_descriptors = (
                self._akaze_templates[label]
            )
            pairs = matcher.knnMatch(
                template_descriptors, scene_descriptors, k=2
            )
            good = [
                first
                for pair in pairs
                if len(pair) == 2
                for first, second in [pair]
                if first.distance
                < self.config.akaze_ratio_test * second.distance
            ]
            if len(good) < 4:
                continue

            source = np.float32(
                [template_keypoints[item.queryIdx].pt for item in good]
            ).reshape(-1, 1, 2)
            destination = np.float32(
                [scene_keypoints[item.trainIdx].pt for item in good]
            ).reshape(-1, 1, 2)
            transform, mask = cv2.findHomography(
                source,
                destination,
                cv2.RANSAC,
                self.config.akaze_ransac_reproj_px,
            )
            if transform is None or mask is None:
                continue

            template_height, template_width = template_shape
            template_corners = np.float32(
                [[
                    [0, 0],
                    [template_width - 1, 0],
                    [template_width - 1, template_height - 1],
                    [0, template_height - 1],
                ]]
            )
            corners = cv2.perspectiveTransform(
                template_corners, transform
            ).reshape(4, 2)
            corners *= scale_back
            quality = self._quad_quality(corners, frame_area)
            if quality is None:
                continue

            inliers = int(mask.sum())
            inlier_ratio = float(inliers) / len(good)
            ranked.append(
                (
                    inliers,
                    len(good),
                    inlier_ratio,
                    label,
                    quality[1],
                    order_quad(corners),
                )
            )

        if not ranked:
            return MatchResult(reason="akaze_no_geometric_match")
        ranked.sort(key=lambda item: (item[0], item[2]), reverse=True)
        best = ranked[0]
        second_inliers = ranked[1][0] if len(ranked) > 1 else 0
        if best[1] < self.config.akaze_min_good_matches:
            return MatchResult(reason="akaze_too_few_matches")
        if best[0] < self.config.akaze_min_inliers:
            return MatchResult(reason="akaze_too_few_inliers")
        if best[2] < self.config.akaze_min_inlier_ratio:
            return MatchResult(reason="akaze_inlier_ratio_too_low")
        if best[0] - second_inliers < self.config.akaze_min_inlier_margin:
            return MatchResult(reason="akaze_inlier_margin_too_low")

        patch = self._warp_candidate(frame, best[5])
        result = self.classify_patch(patch)
        result.target_side_px = best[4]
        result.corners = best[5]
        if result.valid and result.label != best[3]:
            result.valid = False
            result.label = "unknown"
            result.reason = "akaze_label_disagreement"
        elif result.valid:
            result.reason = "accepted_akaze"
        return result

    def match_frame(self, frame: np.ndarray) -> MatchResult:
        if frame is None or frame.size == 0:
            return MatchResult(reason="empty_frame")

        orb_result = self._match_orb_fallback(frame)
        if orb_result.valid:
            return orb_result
        akaze_result = self._match_akaze_fallback(frame)
        if akaze_result.valid:
            return akaze_result

        candidates = self.find_square_candidates(frame)
        if not candidates:
            if akaze_result.reason != "akaze_no_geometric_match":
                return akaze_result
            if orb_result.reason != "orb_no_geometric_match":
                return orb_result
            return akaze_result

        accepted: List[MatchResult] = []
        rejected: List[MatchResult] = []
        for _, side_px, corners in candidates:
            if side_px < self.config.min_target_side_px:
                rejected.append(
                    MatchResult(
                        target_side_px=side_px,
                        corners=corners,
                        reason="target_too_small",
                    )
                )
                continue
            patch = self._warp_candidate(frame, corners)
            result = self.classify_patch(patch)
            result.target_side_px = side_px
            result.corners = corners
            if result.valid:
                accepted.append(result)
            else:
                rejected.append(result)

        if not accepted:
            if rejected:
                best_rejected = max(rejected, key=lambda item: item.score)
                best_rejected.label = "unknown"
                return best_rejected
            if akaze_result.reason != "akaze_no_geometric_match":
                return akaze_result
            if orb_result.reason != "orb_no_geometric_match":
                return orb_result
            return akaze_result

        accepted.sort(key=lambda item: item.score, reverse=True)
        if len(accepted) > 1:
            accepted[0].candidate_margin = (
                accepted[0].score - accepted[1].score
            )
            if accepted[0].candidate_margin < self.config.min_candidate_margin:
                return MatchResult(
                    score=accepted[0].score,
                    second_score=accepted[1].score,
                    candidate_margin=accepted[0].candidate_margin,
                    reason="candidate_margin_too_low",
                )
        else:
            accepted[0].candidate_margin = 1.0
        return accepted[0]

    @staticmethod
    def annotate(
        frame: np.ndarray, result: MatchResult, stable_label: Optional[str] = None
    ) -> np.ndarray:
        output = frame.copy()
        if result.corners.shape == (4, 2):
            corners = np.rint(result.corners).astype(np.int32)
            color = (0, 200, 0) if result.valid else (0, 165, 255)
            cv2.polylines(output, [corners], True, color, 2, cv2.LINE_AA)
        text = (
            f"current={result.label} stable={stable_label or 'none'} "
            f"score={result.score:.3f} margin={result.margin:.3f} "
            f"sharp={result.sharpness:.1f} reason={result.reason}"
        )
        cv2.putText(
            output,
            text,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return output

    @staticmethod
    def annotate_recording(
        frame: np.ndarray,
        result: Optional[MatchResult],
        timestamp_text: str,
    ) -> np.ndarray:
        """Draw only the stable class, capture time and target quadrilateral."""
        output = frame.copy()
        label = "unknown"
        if result is not None and result.valid:
            label = result.label
            if result.corners.shape == (4, 2):
                corners = np.rint(result.corners).astype(np.int32)
                cv2.polylines(
                    output, [corners], True, (0, 255, 0), 3, cv2.LINE_AA
                )

        lines = (f"CLASS: {label}", f"TIME: {timestamp_text}")
        for index, text in enumerate(lines):
            position = (18, 36 + index * 36)
            cv2.putText(
                output,
                text,
                position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 0),
                5,
                cv2.LINE_AA,
            )
            cv2.putText(
                output,
                text,
                position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return output

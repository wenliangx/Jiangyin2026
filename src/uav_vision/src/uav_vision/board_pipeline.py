"""OpenVINO board detection followed by geometry-aware template matching."""

from dataclasses import dataclass, replace
import os
import time
from typing import Any, Dict, List, Mapping, Optional, Tuple

import cv2
import numpy as np

from uav_vision.target_matcher import MatchResult, MatcherConfig, TargetMatcher


@dataclass(frozen=True)
class DetectorConfig:
    input_size: int = 512
    confidence: float = 0.10
    processing_fps: float = 10.0
    inference_threads: int = 4
    opencv_threads: int = 1
    roi_padding: float = 0.75
    direct_min_score: float = 0.63
    direct_min_margin: float = 0.06
    roi_min_target_side_px: float = 40.0
    matcher_max_width: int = 1280

    def __post_init__(self):
        if self.input_size <= 0:
            raise ValueError("detector input_size must be positive")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("detector confidence must be in [0, 1]")
        if self.processing_fps <= 0.0:
            raise ValueError("detector processing_fps must be positive")
        if self.inference_threads <= 0 or self.opencv_threads <= 0:
            raise ValueError("detector thread counts must be positive")
        if self.roi_padding < 0.0:
            raise ValueError("detector roi_padding must be non-negative")
        if not 0.0 <= self.direct_min_score <= 1.0:
            raise ValueError("direct_min_score must be in [0, 1]")
        if not 0.0 <= self.direct_min_margin <= 1.0:
            raise ValueError("direct_min_margin must be in [0, 1]")
        if self.roi_min_target_side_px <= 0.0:
            raise ValueError("roi_min_target_side_px must be positive")
        if self.matcher_max_width <= 0:
            raise ValueError("matcher_max_width must be positive")

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "DetectorConfig":
        allowed = {item.name for item in cls.__dataclass_fields__.values()}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"unknown detector settings: {sorted(unknown)}")
        return cls(**dict(values))


@dataclass(frozen=True)
class Detection:
    box: Tuple[float, float, float, float]
    confidence: float


@dataclass
class PipelineResult:
    match: MatchResult
    detection: Optional[Detection] = None
    method: str = "none"
    detector_ms: float = 0.0
    classification_ms: float = 0.0

    @property
    def total_ms(self) -> float:
        return self.detector_ms + self.classification_ms


class OpenVinoBoardDetector:
    """Small single-class detector used to localize the competition board."""

    def __init__(self, model_path: str, config: DetectorConfig):
        model_path = os.path.abspath(os.path.expanduser(str(model_path)))
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"OpenVINO target detector model does not exist: {model_path}"
            )
        try:
            from openvino import Core
        except ImportError as error:
            raise RuntimeError(
                "OpenVINO Python runtime is unavailable; install/provision "
                "openvino before starting uav_vision"
            ) from error

        self.config = config
        self.model_path = model_path
        core = Core()
        model = core.read_model(model_path)
        expected = tuple(int(value) for value in model.input(0).shape)
        requested = (1, 3, config.input_size, config.input_size)
        if expected != requested:
            raise ValueError(
                f"detector input is {expected}, expected {requested}"
            )
        self._compiled = core.compile_model(
            model,
            "CPU",
            {
                "PERFORMANCE_HINT": "LATENCY",
                "NUM_STREAMS": "1",
                "INFERENCE_NUM_THREADS": config.inference_threads,
            },
        )
        self._output = self._compiled.output(0)
        # Compile and warm the detector while the camera is still gated off so
        # the first enabled image does not pay one-time initialization latency.
        warmup = np.zeros(requested, dtype=np.float32)
        self._compiled([warmup])

    def predict(self, frame: np.ndarray) -> Tuple[List[Detection], float]:
        started = time.perf_counter()
        height, width = frame.shape[:2]
        size = self.config.input_size
        scale = min(size / width, size / height)
        resized_width = max(1, int(round(width * scale)))
        resized_height = max(1, int(round(height * scale)))
        resized = cv2.resize(
            frame,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )
        left = (size - resized_width) // 2
        top = (size - resized_height) // 2
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        canvas[top : top + resized_height, left : left + resized_width] = resized
        input_tensor = np.ascontiguousarray(
            cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
            .transpose(2, 0, 1)[None]
            .astype(np.float32)
            / 255.0
        )

        output = self._compiled([input_tensor])[self._output][0]
        detections = []
        for row in output:
            confidence = float(row[4])
            if confidence < self.config.confidence:
                continue
            x1 = float(np.clip((row[0] - left) / scale, 0, width - 1))
            y1 = float(np.clip((row[1] - top) / scale, 0, height - 1))
            x2 = float(np.clip((row[2] - left) / scale, 0, width - 1))
            y2 = float(np.clip((row[3] - top) / scale, 0, height - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append(
                Detection((x1, y1, x2, y2), confidence)
            )
        detections.sort(key=lambda item: item.confidence, reverse=True)
        # The mission presents one board at a time. Keeping the strongest box
        # bounds classification cost and rejects duplicate end-to-end boxes.
        return detections[:1], (time.perf_counter() - started) * 1000.0


class TargetRecognitionPipeline:
    """Detector cascade that preserves the legacy MatchResult interface."""

    def __init__(
        self,
        detector_config: DetectorConfig,
        matcher_config: MatcherConfig,
        templates_dir: str,
        model_path: str,
    ):
        cv2.setNumThreads(detector_config.opencv_threads)
        cv2.ocl.setUseOpenCL(False)
        self.config = detector_config
        self.detector = OpenVinoBoardDetector(model_path, detector_config)
        direct_config = replace(
            matcher_config,
            min_class_score=detector_config.direct_min_score,
            min_class_margin=detector_config.direct_min_margin,
        )
        roi_config = replace(
            matcher_config,
            min_target_side_px=detector_config.roi_min_target_side_px,
            orb_max_width=detector_config.matcher_max_width,
            akaze_max_width=detector_config.matcher_max_width,
        )
        self.direct_matcher = TargetMatcher(direct_config, templates_dir)
        self.roi_matcher = TargetMatcher(roi_config, templates_dir)

    @staticmethod
    def _box_corners(detection: Detection) -> np.ndarray:
        x1, y1, x2, y2 = detection.box
        return np.asarray(
            ((x1, y1), (x2, y1), (x2, y2), (x1, y2)),
            dtype=np.float32,
        )

    @staticmethod
    def _clip_crop(
        frame: np.ndarray,
        box: Tuple[float, float, float, float],
        padding: float,
    ) -> Tuple[np.ndarray, int, int]:
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = box
        side = max(x2 - x1, y2 - y1)
        amount = padding * side
        crop_x1 = max(0, int(np.floor(x1 - amount)))
        crop_y1 = max(0, int(np.floor(y1 - amount)))
        crop_x2 = min(width, int(np.ceil(x2 + amount)))
        crop_y2 = min(height, int(np.ceil(y2 + amount)))
        return (
            frame[crop_y1:crop_y2, crop_x1:crop_x2],
            crop_x1,
            crop_y1,
        )

    @staticmethod
    def _move_to_frame(result: MatchResult, x_offset: int, y_offset: int):
        if result.corners.shape == (4, 2):
            result.corners = result.corners + np.asarray(
                (x_offset, y_offset), dtype=np.float32
            )
        return result

    def _classify(
        self, frame: np.ndarray, detection: Detection
    ) -> Tuple[MatchResult, str]:
        roi, roi_x, roi_y = self._clip_crop(
            frame, detection.box, self.config.roi_padding
        )
        roi_result = self.roi_matcher.match_frame(roi)
        if roi_result.valid:
            return self._move_to_frame(roi_result, roi_x, roi_y), "roi_geometry"

        crop, _, _ = self._clip_crop(frame, detection.box, 0.0)
        result = self.direct_matcher.classify_patch(crop)
        result.corners = self._box_corners(detection)
        x1, y1, x2, y2 = detection.box
        result.target_side_px = max(x2 - x1, y2 - y1)
        if result.valid:
            result.reason = "accepted_direct_crop"
        return result, "direct_crop"

    def process(self, frame: np.ndarray) -> PipelineResult:
        detections, detector_ms = self.detector.predict(frame)
        if not detections:
            return PipelineResult(
                match=MatchResult(reason="detector_no_target"),
                detector_ms=detector_ms,
            )

        detection = detections[0]
        started = time.perf_counter()
        result, method = self._classify(frame, detection)
        classification_ms = (time.perf_counter() - started) * 1000.0
        return PipelineResult(
            match=result,
            detection=detection,
            method=method,
            detector_ms=detector_ms,
            classification_ms=classification_ms,
        )

    @staticmethod
    def annotate(
        frame: np.ndarray,
        pipeline_result: PipelineResult,
        stable_label: Optional[str] = None,
    ) -> np.ndarray:
        output = TargetMatcher.annotate(
            frame, pipeline_result.match, stable_label
        )
        confidence = (
            pipeline_result.detection.confidence
            if pipeline_result.detection is not None
            else 0.0
        )
        text = (
            f"det={confidence:.3f} method={pipeline_result.method} "
            f"pipeline={pipeline_result.total_ms:.1f}ms"
        )
        cv2.putText(
            output,
            text,
            (15, 58),
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
        return TargetMatcher.annotate_recording(
            frame, result, timestamp_text
        )

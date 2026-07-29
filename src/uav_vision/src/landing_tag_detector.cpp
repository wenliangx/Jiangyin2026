#include "uav_vision/landing_tag_detector.hpp"

#include <stdexcept>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
}

namespace uav_vision {

struct LandingTagDetector::Impl {
  explicit Impl(const DetectorConfig& config) {
    if (config.tag_family != "tag36h11") {
      throw std::invalid_argument("only tag36h11 is supported");
    }
    if (config.threads <= 0 || config.decimate < 1.0) {
      throw std::invalid_argument("invalid AprilTag detector configuration");
    }

    family = tag36h11_create();
    detector = apriltag_detector_create();
    if (family == nullptr || detector == nullptr) {
      throw std::runtime_error("failed to create AprilTag detector");
    }
    apriltag_detector_add_family_bits(detector, family, 2);
    detector->nthreads = config.threads;
    detector->quad_decimate = config.decimate;
    detector->quad_sigma = config.blur;
    detector->refine_edges = config.refine_edges;
  }

  ~Impl() {
    if (detector != nullptr) {
      apriltag_detector_destroy(detector);
    }
    if (family != nullptr) {
      tag36h11_destroy(family);
    }
  }

  apriltag_family_t* family = nullptr;
  apriltag_detector_t* detector = nullptr;
};

LandingTagDetector::LandingTagDetector(const DetectorConfig& config)
    : impl_(new Impl(config)) {}

LandingTagDetector::~LandingTagDetector() = default;

std::vector<TagObservation> LandingTagDetector::detect(const cv::Mat& gray) {
  if (gray.empty() || gray.type() != CV_8UC1) {
    throw std::invalid_argument("AprilTag input must be non-empty mono8");
  }

  const cv::Mat image = gray.isContinuous() ? gray : gray.clone();
  image_u8_t input{
      image.cols,
      image.rows,
      static_cast<int>(image.step),
      const_cast<uint8_t*>(image.ptr<uint8_t>())};
  zarray_t* detections =
      apriltag_detector_detect(impl_->detector, &input);

  std::vector<TagObservation> output;
  output.reserve(zarray_size(detections));
  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t* detection = nullptr;
    zarray_get(detections, i, &detection);

    TagObservation observation;
    observation.id = detection->id;
    observation.center =
        cv::Point2d(detection->c[0], detection->c[1]);
    observation.hamming = detection->hamming;
    observation.decision_margin = detection->decision_margin;
    for (int corner = 0; corner < 4; ++corner) {
      observation.corners[corner] =
          cv::Point2d(detection->p[corner][0], detection->p[corner][1]);
    }
    output.push_back(observation);
  }
  apriltag_detections_destroy(detections);
  return output;
}

}  // namespace uav_vision

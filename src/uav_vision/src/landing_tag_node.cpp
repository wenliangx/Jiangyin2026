#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <uav_vision_msgs/LandingOffset.h>

#include "uav_vision/landing_debug_overlay.hpp"
#include "uav_vision/landing_offset_estimator.hpp"
#include "uav_vision/landing_tag_detector.hpp"

namespace uav_vision {

class LandingTagNode {
 public:
  LandingTagNode()
      : nh_(), pnh_("~"), image_transport_(nh_) {
    DetectorConfig detector_config;
    pnh_.param(
        "detector/tag_family",
        detector_config.tag_family,
        detector_config.tag_family);
    pnh_.param(
        "detector/threads",
        detector_config.threads,
        detector_config.threads);
    pnh_.param(
        "detector/decimate",
        detector_config.decimate,
        detector_config.decimate);
    pnh_.param(
        "detector/blur", detector_config.blur, detector_config.blur);
    pnh_.param(
        "detector/refine_edges",
        detector_config.refine_edges,
        detector_config.refine_edges);

    EstimatorConfig estimator_config;
    XmlRpc::XmlRpcValue ids;
    if (pnh_.getParam("detector/expected_ids", ids)) {
      if (ids.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::invalid_argument("detector/expected_ids must be an array");
      }
      estimator_config.expected_ids.clear();
      for (int index = 0; index < ids.size(); ++index) {
        estimator_config.expected_ids.push_back(static_cast<int>(ids[index]));
      }
    }
    pnh_.param(
        "detector/max_hamming",
        estimator_config.max_hamming,
        estimator_config.max_hamming);
    pnh_.param(
        "detector/min_decision_margin",
        estimator_config.min_decision_margin,
        estimator_config.min_decision_margin);
    pnh_.param(
        "filter/window",
        estimator_config.filter_window,
        estimator_config.filter_window);
    pnh_.param(
        "filter/max_pixel_jump",
        estimator_config.max_pixel_jump,
        estimator_config.max_pixel_jump);
    pnh_.param(
        "filter/reset_frames",
        estimator_config.reset_frames,
        estimator_config.reset_frames);
    expected_ids_ = estimator_config.expected_ids;

    pnh_.param("publish_debug_image", publish_debug_image_, true);
    pnh_.param<std::string>(
        "image_topic", image_topic_, "/vision/down/image_raw");
    pnh_.param<std::string>(
        "result_topic", result_topic_, "/vision/landing/offset");
    pnh_.param<std::string>(
        "debug_topic", debug_topic_, "/vision/landing/debug_image");
    detector_.reset(new LandingTagDetector(detector_config));
    estimator_.reset(new LandingOffsetEstimator(estimator_config));
    result_publisher_ =
        nh_.advertise<uav_vision_msgs::LandingOffset>(result_topic_, 1);
    if (publish_debug_image_) {
      debug_publisher_ = image_transport_.advertise(debug_topic_, 1);
    }
    image_subscriber_ = image_transport_.subscribe(
        image_topic_, 1, &LandingTagNode::imageCallback, this);

    ROS_INFO(
        "landing_tag_node ready: image=%s result=%s",
        image_topic_.c_str(),
        result_topic_.c_str());
  }

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr& image_message) {
    std::vector<TagObservation> observations;
    LandingEstimate estimate;

    try {
      const auto gray = cv_bridge::toCvShare(
          image_message, sensor_msgs::image_encodings::MONO8);
      observations = detector_->detect(gray->image);
      estimate = estimator_->update(
          image_message->width, image_message->height, observations);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "landing image rejected: %s", error.what());
      estimate = estimator_->update(
          image_message->width, image_message->height, {});
    }

    uav_vision_msgs::LandingOffset output;
    output.header = image_message->header;
    output.valid = estimate.valid;
    output.dx = static_cast<float>(estimate.dx);
    output.dy = static_cast<float>(estimate.dy);
    output.center_x = static_cast<float>(estimate.center_x);
    output.center_y = static_cast<float>(estimate.center_y);
    output.tag_ids.assign(estimate.tag_ids.begin(), estimate.tag_ids.end());
    output.tag_count = static_cast<uint8_t>(output.tag_ids.size());
    result_publisher_.publish(output);

    if (publish_debug_image_) {
      publishDebug(image_message, observations, estimate);
    }
  }

  void publishDebug(
      const sensor_msgs::ImageConstPtr& message,
      const std::vector<TagObservation>& observations,
      const LandingEstimate& estimate) {
    try {
      cv::Mat debug = cv_bridge::toCvCopy(
          message, sensor_msgs::image_encodings::BGR8)->image;
      for (const auto& observation : observations) {
        for (int corner = 0; corner < 4; ++corner) {
          cv::line(
              debug,
              observation.corners[corner],
              observation.corners[(corner + 1) % 4],
              cv::Scalar(0, 255, 0),
              2);
        }
        cv::circle(debug, observation.center, 4, cv::Scalar(0, 0, 255), -1);
        std::ostringstream label;
        label << "id=" << observation.id << " h=" << observation.hamming
              << " m=" << static_cast<int>(observation.decision_margin);
        cv::putText(
            debug,
            label.str(),
            observation.center + cv::Point2d(6, -6),
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(0, 255, 255),
            1,
            cv::LINE_AA);
      }

      const cv::Point2d image_center(
          message->width / 2.0, message->height / 2.0);
      cv::drawMarker(
          debug, image_center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
      if (estimate.valid) {
        const cv::Point2d landing_center(
            estimate.center_x, estimate.center_y);
        cv::drawMarker(
            debug,
            landing_center,
            cv::Scalar(0, 0, 255),
            cv::MARKER_CROSS,
            24,
            2);
        cv::line(
            debug, image_center, landing_center, cv::Scalar(255, 0, 255), 2);
      }

      const std::vector<std::pair<std::string, cv::Scalar>> status_lines = {
          {(estimate.valid ? "VALID " : "INVALID ") + estimate.reason,
           estimate.valid ? cv::Scalar(0, 255, 0)
                          : cv::Scalar(0, 0, 255)},
          {formatTagSummary(expected_ids_, observations),
           cv::Scalar(0, 255, 255)},
          {formatPixelOffset(estimate), cv::Scalar(255, 0, 255)},
      };
      int text_y = 30;
      for (const auto& line : status_lines) {
        cv::putText(
            debug,
            line.first,
            cv::Point(16, text_y),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 0, 0),
            4,
            cv::LINE_AA);
        cv::putText(
            debug,
            line.first,
            cv::Point(16, text_y),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            line.second,
            2,
            cv::LINE_AA);
        text_y += 30;
      }
      debug_publisher_.publish(cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8, debug)
                                   .toImageMsg());
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "failed to publish landing debug image: %s",
                        error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher debug_publisher_;
  ros::Publisher result_publisher_;
  std::unique_ptr<LandingTagDetector> detector_;
  std::unique_ptr<LandingOffsetEstimator> estimator_;
  bool publish_debug_image_ = true;
  std::vector<int> expected_ids_;
  std::string image_topic_;
  std::string result_topic_;
  std::string debug_topic_;
};

}  // namespace uav_vision

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_tag_node");
  try {
    uav_vision::LandingTagNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("landing_tag_node failed to start: %s", error.what());
    return 1;
  }
  return 0;
}

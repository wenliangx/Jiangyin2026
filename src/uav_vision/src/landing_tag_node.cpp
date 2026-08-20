#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <uav_vision_msgs/LandingOffset.h>

#include "uav_vision/landing_debug_overlay.hpp"
#include "uav_vision/landing_frame_transformer.hpp"
#include "uav_vision/landing_offset_estimator.hpp"
#include "uav_vision/landing_tag_detector.hpp"

namespace uav_vision {
namespace {

double xmlNumber(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::invalid_argument("pose/camera_to_body entries must be numbers");
}

std::array<double, 9> loadCameraToBody(ros::NodeHandle* private_node) {
  std::array<double, 9> matrix{{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  }};
  XmlRpc::XmlRpcValue configured;
  if (!private_node->getParam("pose/camera_to_body", configured)) {
    return matrix;
  }
  if (configured.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      configured.size() != 9) {
    throw std::invalid_argument(
        "pose/camera_to_body must contain nine row-major values");
  }
  for (int index = 0; index < configured.size(); ++index) {
    matrix[index] = xmlNumber(configured[index]);
  }
  return matrix;
}

bool loadCameraToBodyRpy(
    ros::NodeHandle* private_node,
    std::array<double, 3>* rpy_degrees) {
  XmlRpc::XmlRpcValue configured;
  if (!private_node->getParam(
          "pose/camera_to_body_rpy_deg", configured)) {
    return false;
  }
  if (configured.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      configured.size() != 3) {
    throw std::invalid_argument(
        "pose/camera_to_body_rpy_deg must contain [roll, pitch, yaw]");
  }
  for (int index = 0; index < configured.size(); ++index) {
    (*rpy_degrees)[index] = xmlNumber(configured[index]);
  }
  return true;
}

}  // namespace

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
        "detector/center_id",
        estimator_config.center_id,
        estimator_config.center_id);
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
        "filter/loss_frames",
        estimator_config.loss_frames,
        estimator_config.loss_frames);
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
    pnh_.param<std::string>(
        "pose/topic", pose_topic_, "/mavros/local_position/pose");
    pnh_.param<std::string>(
        "pose/output_frame", output_frame_, "uav_down_camera_level");
    pnh_.param(
        "pose/max_age_seconds", max_pose_age_seconds_, 0.2);
    pnh_.param(
        "pose/focal_length_px", focal_length_px_, 640.0);
    if (max_pose_age_seconds_ <= 0.0) {
      throw std::invalid_argument("pose/max_age_seconds must be positive");
    }
    if (focal_length_px_ <= 0.0) {
      throw std::invalid_argument("pose/focal_length_px must be positive");
    }
    detector_.reset(new LandingTagDetector(detector_config));
    estimator_.reset(new LandingOffsetEstimator(estimator_config));
    if (loadCameraToBodyRpy(&pnh_, &camera_to_body_rpy_deg_)) {
      camera_to_body_ = cameraToBodyFromRpyDegrees(
          camera_to_body_rpy_deg_[0],
          camera_to_body_rpy_deg_[1],
          camera_to_body_rpy_deg_[2]);
      has_rpy_extrinsic_ = true;
    } else {
      camera_to_body_ = loadCameraToBody(&pnh_);
    }
    frame_transformer_.reset(new LandingFrameTransformer(camera_to_body_));
    result_publisher_ =
        nh_.advertise<uav_vision_msgs::LandingOffset>(result_topic_, 1);
    if (publish_debug_image_) {
      debug_publisher_ = image_transport_.advertise(debug_topic_, 1);
    }
    image_subscriber_ = image_transport_.subscribe(
        image_topic_, 1, &LandingTagNode::imageCallback, this);
    pose_subscriber_ = nh_.subscribe(
        pose_topic_, 10, &LandingTagNode::poseCallback, this);
    extrinsic_timer_ = nh_.createTimer(
        ros::Duration(0.2),
        &LandingTagNode::extrinsicTimerCallback,
        this);

    ROS_INFO(
        "landing_tag_node ready: image=%s pose=%s result=%s",
        image_topic_.c_str(),
        pose_topic_.c_str(),
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
    const PlanarOffset leveled_offset = transformToLeveledImage(
        image_message->header.stamp, estimate);
    output.valid = estimate.valid;
    if (output.valid) {
      if (leveled_offset.valid) {
        output.dx = static_cast<float>(leveled_offset.x);
        output.dy = static_cast<float>(leveled_offset.y);
        output.header.frame_id = output_frame_;
      } else {
        output.dx = static_cast<float>(estimate.dx);
        output.dy = static_cast<float>(estimate.dy);
        ROS_WARN_THROTTLE(
            1.0,
            "no fresh fused pose on %s; publishing untransformed pixel offset",
            pose_topic_.c_str());
      }
      output.center_x = static_cast<float>(estimate.center_x);
      output.center_y = static_cast<float>(estimate.center_y);
    } else {
      const float nan = std::numeric_limits<float>::quiet_NaN();
      output.dx = nan;
      output.dy = nan;
      output.center_x = nan;
      output.center_y = nan;
    }
    output.tag_ids.assign(estimate.tag_ids.begin(), estimate.tag_ids.end());
    output.tag_count = static_cast<uint8_t>(output.tag_ids.size());
    result_publisher_.publish(output);

    if (publish_debug_image_) {
      publishDebug(
          image_message, observations, estimate, output, leveled_offset.valid);
    }
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    latest_attitude_.w = message->pose.orientation.w;
    latest_attitude_.x = message->pose.orientation.x;
    latest_attitude_.y = message->pose.orientation.y;
    latest_attitude_.z = message->pose.orientation.z;
    latest_pose_stamp_ = message->header.stamp.isZero()
                             ? ros::Time::now()
                             : message->header.stamp;
    has_pose_ = true;
  }

  void extrinsicTimerCallback(const ros::TimerEvent&) {
    try {
      std::array<double, 3> configured_rpy{{0.0, 0.0, 0.0}};
      if (!loadCameraToBodyRpy(&pnh_, &configured_rpy)) {
        return;
      }
      if (has_rpy_extrinsic_ && configured_rpy == camera_to_body_rpy_deg_) {
        return;
      }
      const auto configured_matrix = cameraToBodyFromRpyDegrees(
          configured_rpy[0], configured_rpy[1], configured_rpy[2]);
      camera_to_body_rpy_deg_ = configured_rpy;
      camera_to_body_ = configured_matrix;
      frame_transformer_.reset(
          new LandingFrameTransformer(camera_to_body_));
      has_rpy_extrinsic_ = true;
      ROS_INFO(
          "updated camera-to-body extrinsic rpy=(%.1f, %.1f, %.1f) deg",
          configured_rpy[0],
          configured_rpy[1],
          configured_rpy[2]);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(
          1.0, "camera-to-body extrinsic update rejected: %s", error.what());
    }
  }

  PlanarOffset transformToLeveledImage(
      const ros::Time& image_stamp,
      const LandingEstimate& estimate) const {
    if (!estimate.valid || !has_pose_) {
      return PlanarOffset{};
    }
    const ros::Time reference_stamp =
        image_stamp.isZero() ? ros::Time::now() : image_stamp;
    if (std::abs((reference_stamp - latest_pose_stamp_).toSec()) >
        max_pose_age_seconds_) {
      return PlanarOffset{};
    }
    return frame_transformer_->cameraToLeveledImage(
        estimate.dx, estimate.dy, focal_length_px_, latest_attitude_);
  }

  std::string poseDebugLine(const ros::Time& image_stamp) const {
    if (!has_pose_) {
      return "pose: MISSING -> RAW fallback";
    }

    const ros::Time reference_stamp =
        image_stamp.isZero() ? ros::Time::now() : image_stamp;
    const double age =
        std::abs((reference_stamp - latest_pose_stamp_).toSec());
    const double norm = std::sqrt(
        latest_attitude_.w * latest_attitude_.w +
        latest_attitude_.x * latest_attitude_.x +
        latest_attitude_.y * latest_attitude_.y +
        latest_attitude_.z * latest_attitude_.z);
    if (!std::isfinite(norm) || norm < 1e-9) {
      return "pose: INVALID quaternion -> RAW fallback";
    }

    const double w = latest_attitude_.w / norm;
    const double x = latest_attitude_.x / norm;
    const double y = latest_attitude_.y / norm;
    const double z = latest_attitude_.z / norm;
    const double roll = std::atan2(
        2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch_sine = std::max(
        -1.0, std::min(1.0, 2.0 * (w * y - z * x)));
    const double pitch = std::asin(pitch_sine);
    const double yaw = std::atan2(
        2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    constexpr double kRadiansToDegrees = 57.29577951308232;

    std::ostringstream line;
    const bool pose_fresh = age <= max_pose_age_seconds_;
    line << std::fixed << std::setprecision(1)
         << "pose: " << (pose_fresh ? "FRESH" : "STALE")
         << " age=" << age * 1000.0 << "ms rpy=("
         << roll * kRadiansToDegrees << ","
         << pitch * kRadiansToDegrees << ","
         << yaw * kRadiansToDegrees << ")deg";
    if (!pose_fresh) {
      line << " -> RAW fallback";
    }
    return line.str();
  }

  std::string matrixDebugLine(int row) const {
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "R_body_camera[" << row << "]: "
         << std::showpos
         << camera_to_body_[row * 3] << " "
         << camera_to_body_[row * 3 + 1] << " "
         << camera_to_body_[row * 3 + 2];
    return line.str();
  }

  std::string extrinsicRpyDebugLine() const {
    if (!has_rpy_extrinsic_) {
      return "extrinsic rpy: legacy matrix";
    }
    std::ostringstream line;
    line << std::fixed << std::setprecision(1)
         << "extrinsic rpy deg: ("
         << camera_to_body_rpy_deg_[0] << ","
         << camera_to_body_rpy_deg_[1] << ","
         << camera_to_body_rpy_deg_[2] << ")";
    return line.str();
  }

  void publishDebug(
      const sensor_msgs::ImageConstPtr& message,
      const std::vector<TagObservation>& observations,
      const LandingEstimate& estimate,
      const uav_vision_msgs::LandingOffset& output,
      bool transformed) {
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
        cv::arrowedLine(
            debug,
            image_center,
            landing_center,
            cv::Scalar(255, 0, 255),
            5,
            cv::LINE_AA,
            0,
            0.12);
      }

      const cv::Scalar output_color =
          transformed ? cv::Scalar(255, 255, 0)
                      : cv::Scalar(0, 255, 255);
      if (output.valid) {
        const cv::Point2d output_tip(
            image_center.x + output.dx, image_center.y + output.dy);
        cv::arrowedLine(
            debug,
            image_center,
            output_tip,
            output_color,
            2,
            cv::LINE_AA,
            0,
            0.18);
        cv::putText(
            debug,
            transformed ? "LEVEL output" : "RAW fallback",
            output_tip + cv::Point2d(8, 18),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            output_color,
            2,
            cv::LINE_AA);
      }

      std::ostringstream output_line;
      output_line << std::fixed << std::setprecision(1)
                  << (transformed ? "OUT(level)" : "OUT(raw fallback)")
                  << " dx=";
      if (output.valid) {
        output_line << std::showpos << output.dx
                    << " dy=" << output.dy << " px";
      } else {
        output_line << "-- dy=-- px";
      }

      const std::vector<std::pair<std::string, cv::Scalar>> status_lines = {
          {(estimate.valid ? "VALID " : "INVALID ") + estimate.reason,
           estimate.valid ? cv::Scalar(0, 255, 0)
                          : cv::Scalar(0, 0, 255)},
          {formatTagSummary(expected_ids_, observations),
           cv::Scalar(0, 255, 255)},
          {"RAW(image) " + formatPixelOffset(estimate),
           cv::Scalar(255, 0, 255)},
          {output_line.str(), output_color},
          {poseDebugLine(message->header.stamp), output_color},
          {extrinsicRpyDebugLine(), cv::Scalar(255, 255, 255)},
          {matrixDebugLine(0), cv::Scalar(255, 255, 255)},
          {matrixDebugLine(1), cv::Scalar(255, 255, 255)},
          {matrixDebugLine(2), cv::Scalar(255, 255, 255)},
          {"virtual focal length: " + std::to_string(focal_length_px_) +
               " px",
           cv::Scalar(255, 255, 255)},
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
        text_y += 28;
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
  ros::Subscriber pose_subscriber_;
  ros::Timer extrinsic_timer_;
  std::unique_ptr<LandingTagDetector> detector_;
  std::unique_ptr<LandingOffsetEstimator> estimator_;
  std::unique_ptr<LandingFrameTransformer> frame_transformer_;
  std::array<double, 9> camera_to_body_{{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  }};
  std::array<double, 3> camera_to_body_rpy_deg_{{0.0, 0.0, 0.0}};
  AttitudeQuaternion latest_attitude_;
  ros::Time latest_pose_stamp_;
  bool has_pose_ = false;
  bool has_rpy_extrinsic_ = false;
  bool publish_debug_image_ = true;
  double max_pose_age_seconds_ = 0.2;
  double focal_length_px_ = 640.0;
  std::vector<int> expected_ids_;
  std::string image_topic_;
  std::string result_topic_;
  std::string debug_topic_;
  std::string pose_topic_;
  std::string output_frame_;
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

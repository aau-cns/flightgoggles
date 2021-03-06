/// Copyright (C) 2018 Winter Guerra.
/// Copyright (C) 2020 Martin Scheiber, Control of Networked Systems,
/// University of Klagenfurt, Austria.
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available in the
/// LICENSE file. No license in patents is granted.
///
/// You can contact the author at martin.scheiber@ieee.org

#ifndef ROSCLIENT_HPP
#define ROSCLIENT_HPP

#include <FlightGogglesClient.hpp>
#include <FisheyeModel.hpp>

#include <iostream>
#include <cmath>
#include <random>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include <ros/time.h>
#include <ros/service_server.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "flightgoggles/IRMarkerArray.h"
#include "flightgoggles/FGParam.h"
#include <flightgoggles/SetTransformStamped.h>
#include <flightgoggles/SetTransformStampedRequest.h>
#include <flightgoggles/SetTransformStampedResponse.h>
#include <flightgoggles/SetTransformStampedArr.h>
#include <flightgoggles/SetTransformStampedArrRequest.h>
#include <flightgoggles/SetTransformStampedArrResponse.h>

#include "geometry_msgs/Pose.h"
#include "tf2_msgs/TFMessage.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <dynamic_reconfigure/server.h>
#include <flightgoggles_ros_bridge/BridgeConfigConfig.h>

class ROSClient {
public:
  /// @name Parameter struct
  struct SimParams {
      std::string     sim_scene_name {"gen3dscene"};  //!< scene name to simulate
      Eigen::Vector2i sim_cam_dim {640, 480};         //!< image dimension to use (only 640, 480)
      float           sim_cam_fov {70};               //!<

      float level_illumination {0.0};
      float level_features {0.0};
      float level_clutter {0.0};
      float level_motion_blur {0.0};

      float cam_distortion {1.0};
      bool  cam_is_grayscale {false};

      float cam_time_delay {0.0};
      float imu_acc_noise {0.0};
      float imu_gyro_noise {0.0};

      uint64_t cam_time_delay_ns() {
        return (uint64_t) (cam_time_delay * 1e9);
      }
  };

  typedef flightgoggles_ros_bridge::BridgeConfigConfig BridgeConfig_t;
  typedef dynamic_reconfigure::Server<BridgeConfig_t> ReconfServer_t;

  /// @name FlightGoggles interface object
	FlightGogglesClient flightGoggles;

  /// @name CameraModel
  FisheyeCamera camera_model_;

	/// @name Node handles for ROS
	//@{
	ros::NodeHandle ns_;
  ros::NodeHandle nsPrivate_;
  //@}

	/// @name Transform listeners for listening to ROS TF
	//@{
	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;
	//@}

	/// @name Subscribers
	//@{
  ros::Subscriber tfSubscriber_;
  ros::Subscriber irSubscriber_;
  ros::Subscriber paramSubscriber_;
  //@}

  /// @name Image publishers
  //@{
	image_transport::ImageTransport it_;
	image_transport::CameraPublisher imagePubLeft_;
	image_transport::CameraPublisher imagePubRight_;
	//@}

	/// @name Topic publishers
	//@{
	ros::Publisher collisionPub_;
  ros::Publisher lidarPub_;
  ros::Publisher irMarkerPub_;
  ros::Publisher fpsPublisher_;
  //@}

  /// @name Service server
  //@{
  ros::ServiceServer setTfSrv_;
  ros::ServiceServer setTfArrSrv_;
  //@}

  //// @name State variables
  //@{
  bool render_stereo = false;
  int numSimulationStepsSinceLastRender_ = 0;
  const int numSimulationStepsBeforeRenderRequest_ = 15;
//    sensor_msgs::PointCloud2::Ptr irBeaconGroundTruth_;
  sensor_msgs::CameraInfo cameraInfoLeft;
  sensor_msgs::CameraInfo cameraInfoRight;
	float baseline_ = 0.32;
	int imageWidth_ = 1024;
	int imageHeight_ = 768;

  // Sim Params
  SimParams sim_parameters_;
  bool b_have_renders_ {false};

  // Scene Params
  ReconfServer_t reconf_server_;
  unity_outgoing::Scene_t scene_params_;

	// Lidar params
  float lidarMaxRange_ = 20; // Meters
  float lidarVariance_ = 0.0009; // Meters^2

  // Noise generators for lidar
  std::default_random_engine randomNumberGenerator_;
  std::normal_distribution<float> standardNormalDistribution_ = std::normal_distribution<float>(0.0,1.0);

  // Keep track of the frame rate
  ros::WallTime timeSinceLastMeasure_;
  int64_t frameCount_;
  //@}

  /// @name Transforms
  //@{
  geometry_msgs::TransformStamped imu_T_Camera_;
  //@}

  /// @name Constructor
	ROSClient(ros::NodeHandle nh, ros::NodeHandle nhPrivate);

	// Populate starting settings into state
	void populateRenderSettings();
  void populateSimulationSettings();


	/// @name Callbacks
	void tfCallback(tf2_msgs::TFMessage::Ptr msg);
//    void irBeaconPointcloudCallback(sensor_msgs::PointCloud2::Ptr msg);
  void paramCallback(const flightgoggles::FGParamConstPtr& msg);
  void dynamicCallback(BridgeConfig_t &config, uint32_t level);


  /// @name Image Transform Methods

  //
  void imageDistortColor(sensor_msgs::ImagePtr& msg, cv::Mat image);

  std::atomic<bool> bIamgeReceived_;
  bool setTfSrvCallback(flightgoggles::SetTransformStampedRequest & request, flightgoggles::SetTransformStampedResponse& response);
  bool setTranformStampedArrSrvCallback(flightgoggles::SetTransformStampedArrRequest & request, flightgoggles::SetTransformStampedArrResponse& response);
private:
  void performCameraTest();
};

#endif

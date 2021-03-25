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

#include "ROSClient.hpp"

#define SHOW_DEBUG_IMAGE_FEED false

/// Constructor
ROSClient::ROSClient(ros::NodeHandle ns, ros::NodeHandle nhPrivate):
    // Node handle
    ns_(ns),
    nsPrivate_(nhPrivate),
    // TF listener
    tfListener_(tfBuffer_),
    //Image transport
    it_(ns_),
    b_have_renders_(false)
{
    // perform camera test, if needed
//    performCameraTest();
//  ROS_FATAL("NEW TEST");

    // Wait for static transforms between imu/cameras.
    try{
        imu_T_Camera_ = tfBuffer_.lookupTransform( "uav/camera/left", "uav/imu", ros::Time(0),
                                                    ros::Duration(10.0));
//        imu_T_Camera_ = tfBuffer_.lookupTransform(  "uav/imu", "uav/camera/left", ros::Time(0),
//                                                   ros::Duration(10.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT find required uav to camera transform: %s", ex.what());
        exit(1);
    }

    // read yaml parameters

    if (!ros::param::get("/uav/flightgoggles_ros_bridge/render_stereo", render_stereo)) {
        //std::cout << "Did not get argument for render_stereo. Defaulting to false" << std::endl;
    }

//    int imageWidth = 1024, imageHeight = 768;
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/image_width", sim_parameters_.sim_cam_dim(0))) {
        //std::cout << "Did not get argument for image width. Defaulting to 1024 px" << std::endl;
    }

    if (!ros::param::get("/uav/flightgoggles_ros_bridge/image_height", sim_parameters_.sim_cam_dim(1))) {
        //std::cout << "Did not get argument for image height. Defaulting to 768 px" << std::endl;
    }

    if (!ros::param::get("/uav/flightgoggles_ros_bridge/baseline", baseline_)) {
        //std::cout << "Did not get argument for baseline. Defaulting to 0.32 m" << std::endl;
    }

    if (!ros::param::get("/uav/flightgoggles_ros_bridge/fov", sim_parameters_.sim_cam_fov)) {
        //std::cout << "Did not get argument for fov. Defaulting to 70 deg." << std::endl;
    }

    if (!ros::param::get("/uav/flightgoggles_laser/rangefinder_max_range", lidarMaxRange_)) {
        //std::cout << "Did not get argument for rangefinder max range. Defaulting to 20 m" << std::endl;
    }

    if (!ros::param::get("/uav/flightgoggles_laser/rangefinder_variance", lidarVariance_)) {
        //std::cout << "Did not get argument for rangefinder variance. Defaulting to 0.009 m^2" << std::endl;
    }

    // check if FOV is in deg or rad
    if (sim_parameters_.sim_cam_fov < M_PI)
    {
      ROS_WARN("Given fov is lower than pi, assuming it is in rad.\nConverted %f [rad] to %f [deg].",
               sim_parameters_.sim_cam_fov,
               sim_parameters_.sim_cam_fov*M_RAD2DEG);
      sim_parameters_.sim_cam_fov *= M_RAD2DEG;
    }

//    // Load params
//    populateRenderSettings();

    // init image publisher
    imagePubLeft_ = it_.advertiseCamera("/uav/camera/left/image_rect_color", 1);
    if(render_stereo) {
        imagePubRight_ = it_.advertiseCamera("/uav/camera/right/image_rect_color", 1);
    }

    // Collision publisher
    collisionPub_ = ns_.advertise<std_msgs::Empty>("/uav/collision", 1);

    // Lidar publisher
    lidarPub_ = ns_.advertise<sensor_msgs::Range>("/uav/sensors/downward_laser_rangefinder", 1);

    // IR Marker publisher
    irMarkerPub_ = ns_.advertise<flightgoggles::IRMarkerArray>("/uav/camera/left/ir_beacons", 1);


    // Subscribe to TF messages
    tfSubscriber_ = ns_.subscribe("/tf", 1, &ROSClient::tfCallback, this, ros::TransportHints().tcpNoDelay());

    // Publish the estimated latency
    fpsPublisher_ = ns_.advertise<std_msgs::Float32>("/uav/camera/debug/fps", 1);


    // Sunscribe to Param messages
    paramSubscriber_ = ns_.subscribe("/uav/params", 1, &ROSClient::paramCallback, this);

    // Subscribe to IR beacon locations
//    irSubscriber_ = ns_.subscribe("/challenge/ir_BeaconsGroundTruth", 1, &ROSClient::irBeaconPointcloudCallback, this);

    // Start Service servers
    setTfSrv_ = ns_.advertiseService("/set_tf_service", &ROSClient::setTfSrvCallback, this);
    setTfArrSrv_ = ns_.advertiseService("/set_tf_array_service", &ROSClient::setTranformStampedArrSrvCallback, this);

    ReconfServer_t::CallbackType f = boost::bind(&ROSClient::dynamicCallback, this, _1, _2);
    reconf_server_.setCallback(f);

    // synchronization flag for services
    bIamgeReceived_ = false;

    // Initialize random seed
    std::srand (time(nullptr));
}

void ROSClient::populateSimulationSettings () {
  // Scene/Render settings
  /*
  Available scenes:
  sceneFilename = "Butterfly_World";
  sceneFilename = "FPS_Warehouse_Day";
  sceneFilename = "FPS_Warehouse_Night";
  sceneFilename = "Hazelwood_Loft_Full_Day";
  sceneFilename = "Hazelwood_Loft_Full_Night";

   // NEW for FlightGoggles v2.x.x
   flightGoggles.state.sceneFilename = "Abandoned_Factory_Morning";
   flightGoggles.state.sceneFilename = "Abandoned_Factory_Sunset";
   */

  flightGoggles.state.sceneFilename   = sim_parameters_.sim_scene_name;
  flightGoggles.state.camWidth        = sim_parameters_.sim_cam_dim(0);
  flightGoggles.state.camHeight       = sim_parameters_.sim_cam_dim(1);
  flightGoggles.state.camFOV          = sim_parameters_.sim_cam_fov;
  flightGoggles.state.camMotionBlur   = sim_parameters_.level_motion_blur;

  // add scene data
  flightGoggles.state.illumination    = sim_parameters_.level_illumination;
  flightGoggles.state.objectClutter   = sim_parameters_.level_clutter;
  flightGoggles.state.objectFeatures  = sim_parameters_.level_features;

  flightGoggles.state.scene           = scene_params_;

  // set up the CameraInfo struct
  // set up the CameraInfo struct, note that since the rendered pixel are assumed
  // to be square fx=fy and then it can be computed directly given the vertical FOV
  // of the camera, pay attention that also in the binary the FOV of the camera
  // needs to be set as vertical FOV
  cameraInfoLeft = {};
  cameraInfoLeft.width = flightGoggles.state.camWidth;
  cameraInfoLeft.height = flightGoggles.state.camHeight;
  cameraInfoLeft.distortion_model = "plum_bob";
  float f = (cameraInfoLeft.height / 2.0) / tan((M_PI * (flightGoggles.state.camFOV / 180.0)) / 2.0);
  float cx = cameraInfoLeft.width / 2.0;
  float cy = cameraInfoLeft.height / 2.0;
  float tx = 0.0;
  float ty = 0.0;
  cameraInfoLeft.D = {0.0, 0.0, 0.0, 0.0, 0.0};
  cameraInfoLeft.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
  cameraInfoLeft.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  cameraInfoLeft.P = {f, 0.0, cx, tx, 0.0, f, cy, ty, 0.0, 0.0, 1.0, 0.0};

  /// @todo initialize camera model here
  // update camera model
  camera_model_.setCameraParameters(
        cameraInfoLeft.width,
        cameraInfoLeft.height,
        f, f, cx, cy,
        sim_parameters_.cam_distortion);

  if (render_stereo) {
      cameraInfoRight = {};
      cameraInfoRight.width = flightGoggles.state.camWidth;
      cameraInfoRight.height = flightGoggles.state.camHeight;
      cameraInfoRight.distortion_model = "plum_bob";
      float f = (cameraInfoRight.height / 2.0) / tan((M_PI * (flightGoggles.state.camFOV / 180.0)) / 2.0);
      float cx = cameraInfoRight.width / 2.0;
      float cy = cameraInfoRight.height / 2.0;
      float tx = -f * baseline_ ; // -fx' * B
      float ty = 0.0;
      cameraInfoRight.D = {0.0, 0.0, 0.0, 0.0, 0.0};
      cameraInfoRight.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
      cameraInfoRight.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      cameraInfoRight.P = {f, 0.0, cx, tx, 0.0, f, cy, ty, 0.0, 0.0, 1.0,
                          0.0};
  }

  ROS_INFO("Updated simulation settings.");
}

void ROSClient::populateRenderSettings()  {
    populateSimulationSettings();

    // check if camera renders have been set up
    if (!b_have_renders_)
    {
      // Prepopulate metadata of cameras
      unity_outgoing::Camera_t cam_RGB_left;
      cam_RGB_left.ID = "Camera_RGB_left";
      cam_RGB_left.channels = 3;
      cam_RGB_left.isDepth = false;
      cam_RGB_left.outputIndex = 0;
      cam_RGB_left.hasCollisionCheck = true;
      cam_RGB_left.doesLandmarkVisCheck = true;
      // Add cameras to persistent state
      flightGoggles.state.cameras.push_back(cam_RGB_left);

      if (render_stereo) {
          unity_outgoing::Camera_t cam_RGB_right;
          cam_RGB_right.ID = "Camera_RGB_right";
          cam_RGB_right.channels = 3;
          cam_RGB_right.isDepth = false;
          cam_RGB_right.outputIndex = 1;
          cam_RGB_right.hasCollisionCheck = false;
          cam_RGB_right.doesLandmarkVisCheck = false;

          flightGoggles.state.cameras.push_back(cam_RGB_right);
      }

      b_have_renders_ = true;

      ROS_INFO("Updated render settings.");
    }

}
// Subscribe to all TF messages to avoid lag.
void ROSClient::tfCallback(tf2_msgs::TFMessage::Ptr msg){
    //geometry_msgs::TransformStamped world_to_uav;
    bool found_transform = false;

    // Check if TF message is for world->uav/imu
    // This is output by dynamics node.
    for (auto transform : msg->transforms){
        if (transform.child_frame_id == "uav/imu"){
            //world_to_uav = transform;
            found_transform = true;
        }
    }

    // Skip if do not have transform
    if (!found_transform) return;

    // Skip every other transform to get an update rate of 60hz
    if (numSimulationStepsSinceLastRender_ >= numSimulationStepsBeforeRenderRequest_){

        // Get transform for left camera
        geometry_msgs::TransformStamped camLeftTransform;

        try{
            camLeftTransform = tfBuffer_.lookupTransform("world", "uav/camera/left/internal_nwu", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could NOT find transform for /uav/camera/left/nwu: %s", ex.what());
        }

        Transform3 camLeftPose = tf2::transformToEigen(camLeftTransform);
        flightGoggles.setCameraPoseUsingROSCoordinates(camLeftPose, 0);

        // Get transform for second camera (right)
        if (render_stereo) {

            geometry_msgs::TransformStamped camRightTransform;

            try{
                camRightTransform = tfBuffer_.lookupTransform("world", "uav/camera/right/internal_nwu", camLeftTransform.header.stamp);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Could NOT find transform for /uav/camera/right/nwu: %s", ex.what());
            }

            Transform3 camRightPose = tf2::transformToEigen(camRightTransform);

            flightGoggles.setCameraPoseUsingROSCoordinates(camRightPose, 1);
        }

        // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
        flightGoggles.state.ntime = camLeftTransform.header.stamp.toNSec();
        // request render
        flightGoggles.requestRender();

        // reset scene reinitialize
        flightGoggles.state.scene.reinitialize &= false;
        scene_params_.reinitialize &= false;

        numSimulationStepsSinceLastRender_ = 0;

    } else {
        numSimulationStepsSinceLastRender_++;
    }

}

/**
 * @brief ROSClient::paramCallback updates the sim parameters which are send to unity.
 *        for illumination we would like to set values from 0.3 to 4 where 1.8 is the
 *        optimal illumination
 * @param msg
 *
 * @todo sim scene settings are not yet saved or transmitted
 */
void ROSClient::paramCallback(const flightgoggles::FGParamConstPtr& msg)
{
  // update current parameter settings
  if (msg->sim_scene_name != "") {
    sim_parameters_.sim_scene_name = msg->sim_scene_name;
  }

  // Set if illumination should decrease only, increase only or be random
  int plus_minus = 0;

  if (msg->illumination_changes_direction.compare("up") == 0) {
    plus_minus = 1;
  } else if (msg->illumination_changes_direction.compare("down") == 0) {
    plus_minus = - 1;
  } else if (msg->illumination_changes_direction.compare("random") == 0) {
    plus_minus = (std::rand() % 2) * 2 - 1;
  } else {
    ROS_ERROR_STREAM("No or wrong Illumination direction parameter, valid options are [up],[down] and [random] ");
    ROS_WARN_STREAM("Seeting it to [random]");
    plus_minus = (std::rand() % 2) * 2 - 1;
  }

  sim_parameters_.sim_cam_dim(0)      = (int) msg->sim_cam_dimensions.x;
  sim_parameters_.sim_cam_dim(1)      = (int) msg->sim_cam_dimensions.y;
  sim_parameters_.sim_cam_fov         = msg->sim_cam_fov;
  sim_parameters_.level_clutter       = msg->level_clutter;
  sim_parameters_.level_features      = msg->level_features;
  sim_parameters_.level_motion_blur   = msg->level_motion_blur;
  sim_parameters_.cam_distortion      = msg->cam_fisheye_distortion;
  sim_parameters_.cam_is_grayscale    = msg->cam_is_grayscale;
  sim_parameters_.imu_acc_noise       = msg->imu_acc_noise;
  sim_parameters_.imu_gyro_noise      = msg->imu_gyro_noise;
  sim_parameters_.cam_time_delay      = msg->cam_time_delay;

  if (plus_minus > 0) {
    // (1/54)l^2 + (1/9)l +1
    sim_parameters_.level_illumination  = 1.0 + (1.0/54)*std::pow(msg->level_illumination,2) + (1.0/9.0)*msg->level_illumination;
  } else if (plus_minus < 0){
    // (0.72/54)l^2 - 0.23l + 1
    sim_parameters_.level_illumination  = 1.0 + (0.73/54)*std::pow(msg->level_illumination,2) - 0.23*msg->level_illumination;
  }

  ROS_INFO_STREAM("Set scene to " << sim_parameters_.sim_scene_name);
  ROS_INFO_STREAM("Set image width to " << sim_parameters_.sim_cam_dim(0));
  ROS_INFO_STREAM("Set image height to " << sim_parameters_.sim_cam_dim(1));
  ROS_INFO_STREAM("Set camera diagonal fov to " << sim_parameters_.sim_cam_fov);
  ROS_INFO_STREAM("Set camera fisheye distortion to " << sim_parameters_.cam_distortion);
  ROS_INFO_STREAM("Set camera to grayscale " << sim_parameters_.cam_is_grayscale);
  ROS_INFO_STREAM("Set illumination (difficulty) level to " << sim_parameters_.level_illumination);
  ROS_INFO_STREAM("Set features (difficulty) level to " << sim_parameters_.level_features);
  ROS_INFO_STREAM("Set motion blur (difficulty) level to " << sim_parameters_.level_motion_blur);
  ROS_INFO_STREAM("Set clutter (difficulty) level to " << sim_parameters_.level_clutter);
  ROS_INFO_STREAM("Set imu accelerometer noise level to " << sim_parameters_.imu_acc_noise);
  ROS_INFO_STREAM("Set imu gyrometer noise level to " << sim_parameters_.imu_gyro_noise);
  ROS_INFO_STREAM("Set cam-imu time delay level to " << sim_parameters_.cam_time_delay);

  // setup scene params to reinitialize
  scene_params_.reinitialize = true;

  // update settings
  populateRenderSettings();
}

void ROSClient::dynamicCallback(BridgeConfig_t &config, uint32_t level)
{
  scene_params_.objectsDensityMultiplier    = config.o_density_multiplier;
  scene_params_.objectsDistributionScale    = config.o_distribution_scale;
  scene_params_.objectsPlacementProbability = config.o_placement_prob;
//  scene_params_.objectsDensity              = config.o_density;
//  scene_params_.objectsPlacement            = config.o_placement;
  scene_params_.objectsPlacementMinFactor   = config.o_placement_min;
  scene_params_.objectsPlacementMaxFactor   = config.o_placement_max;
  scene_params_.objectsDistUniformity       = config.o_dist_uniformity;
  scene_params_.objectsDistOffset           = config.o_dist_offset;
  scene_params_.objectsDistMultiplier       = config.o_dist_multiplier;
  scene_params_.objectOriginRadius          = config.o_origin_radius;

  scene_params_.heightMeshFactor            = config.h_mesh_factor;
  scene_params_.heightDistributionScale     = config.h_dist_scale;
  scene_params_.heightDistUniformity        = config.h_dist_uniformity;
  scene_params_.heightDistOffset            = config.h_dist_offset;
  scene_params_.heightDistMultiplier        = config.h_dist_multiplier;

  // set render settings
  populateRenderSettings();
}

//void ROSClient::irBeaconPointcloudCallback(sensor_msgs::PointCloud2::Ptr msg) {
//    irBeaconGroundTruth_ = msg;
//}

/**
 * @brief ROSClient::checkImageColor
 * @param msg
 * @param image
 */
void ROSClient::imageDistortColor(sensor_msgs::ImagePtr &msg, cv::Mat image)
{
  cv::Mat out;
  std::string img_type;

  if (sim_parameters_.cam_is_grayscale)
  {
    cv::cvtColor(image, out, cv::COLOR_BGR2GRAY);
    img_type = "mono8";
  }
  else
  {
    out = image;
    img_type = "bgr8";
  }

  /// @todo this forced the dimension of out to be in, which creates black corners
  /// should be changed after thesting
  // check if distortion is present
  if (sim_parameters_.cam_distortion != 0.0)
    camera_model_.applyDistortion(out);

  msg = cv_bridge::CvImage(std_msgs::Header(), img_type, out).toImageMsg();

}

/**
 * @brief ROSClient::performCameraTest tests the camera model and distortion.
 *
 * @note This function is private only!!
 */
void ROSClient::performCameraTest()
{
//  int camWidth = 8;
//  int camHeight = 6;
  int camWidth = 1024;
  int camHeight = 768;
//  int camWidth = 640;
//  int camHeight = 480;
  float camFOV = 70.0f;
  float fx = 0.644717*camWidth;
  float fy = 0.966123*camHeight;
  float cx = 0.5 * camWidth;
  float cy = 0.5 * camHeight;
  float s = 0.872665;
//  float s = 1.5;
//  float s = 0.0;
  ROS_INFO("updating camera model");
  camera_model_.setCameraParameters(
        camWidth,
        camHeight,
        fx, fy, cx, cy, s);
  camera_model_.performTest("/home/martin/ros-workspaces/aaucns_workspaces/aaucns_flightgoogles_cws/src/flightgoggles/flightgoggles_ros_bridge/src/CameraModel/test", "image.png");
}

bool ROSClient::setTfSrvCallback(
    flightgoggles::SetTransformStampedRequest &request,
    flightgoggles::SetTransformStampedResponse &response)
{

  // Transform for left camera
  geometry_msgs::TransformStamped camLeftTransform;

  // Get old transform
  try{
      camLeftTransform = tfBuffer_.lookupTransform("world", "uav/camera/left/internal_nwu", ros::Time(0));
  } catch (tf2::TransformException &ex) {
      ROS_WARN("Could NOT find transform for /uav/camera/left/nwu: %s", ex.what());
  }

  // Check if there exist negative time differences (actual transform back in time with respect to buffer)
  if (request.tf.header.stamp.toSec() < camLeftTransform.header.stamp.toSec()) {
    ROS_WARN("Time jump detected, clearing TF buffer");
    tfBuffer_.clear();
  }

  // Get imu transform from service request and add it to tfBuffer
  tfBuffer_.setTransform(request.tf, "");

  // Get new transform
  try{
      camLeftTransform = tfBuffer_.lookupTransform("world", "uav/camera/left/internal_nwu", ros::Time(0));
  } catch (tf2::TransformException &ex) {
      ROS_WARN("Could NOT find transform for /uav/camera/left/nwu: %s", ex.what());
  }

  Transform3 camLeftPose = tf2::transformToEigen(camLeftTransform);
  flightGoggles.setCameraPoseUsingROSCoordinates(camLeftPose, 0);

  // Get transform for second camera (right)
  if (render_stereo) {

      geometry_msgs::TransformStamped camRightTransform;

      try{
          camRightTransform = tfBuffer_.lookupTransform("world", "uav/camera/right/internal_nwu", camLeftTransform.header.stamp);
      } catch (tf2::TransformException &ex) {
          ROS_WARN("Could NOT find transform for /uav/camera/right/nwu: %s", ex.what());
      }

      Transform3 camRightPose = tf2::transformToEigen(camRightTransform);
      flightGoggles.setCameraPoseUsingROSCoordinates(camRightPose, 1);
  }

  // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
  flightGoggles.state.ntime = camLeftTransform.header.stamp.toNSec();

  // Set image received flag to false
  bIamgeReceived_ = false;

  // request render
  flightGoggles.requestRender();

  // reset scene reinitialize
  flightGoggles.state.scene.reinitialize &= false;
  scene_params_.reinitialize &= false;

  // Wait until image ready (polling every 1 us, timeout to 5ms)
  int cnt = 0;
  while(!bIamgeReceived_) {
    usleep(1);
    if (cnt == 5000) {
      // Fail responce
      response.result = false;
      return false;
    }
    ++cnt;
  }

  // Succes responce
  response.result = true;
  return true;
}

bool ROSClient::setTranformStampedArrSrvCallback(
    flightgoggles::SetTransformStampedArrRequest &request,
    flightgoggles::SetTransformStampedArrResponse &response)
{
  // Check that array is not empty to avoid segfault
  if(!request.tf_arr.empty()) {
    for (const auto &it : request.tf_arr) {

      // Transform for left camera
      geometry_msgs::TransformStamped camLeftTransform;

      // Get old transform
      try{
          camLeftTransform = tfBuffer_.lookupTransform("world", "uav/camera/left/internal_nwu", ros::Time(0));
      } catch (tf2::TransformException &ex) {
          ROS_WARN("Could NOT find transform for /uav/camera/left/nwu: %s", ex.what());
      }

      // Check if there exist negative time differences (actual transform back in time with respect to buffer)
      if (it.header.stamp.toSec() < camLeftTransform.header.stamp.toSec()) {
        ROS_WARN("Time jump detected, clearing TF buffer");
        tfBuffer_.clear();
      }

      // Get imu transform from service request and add it to tfBuffer
      tfBuffer_.setTransform(it, "");

      // Get new transform
      try{
          camLeftTransform = tfBuffer_.lookupTransform("world", "uav/camera/left/internal_nwu", ros::Time(0));
      } catch (tf2::TransformException &ex) {
          ROS_WARN("Could NOT find transform for /uav/camera/left/nwu: %s", ex.what());
      }

      Transform3 camLeftPose = tf2::transformToEigen(camLeftTransform);
      flightGoggles.setCameraPoseUsingROSCoordinates(camLeftPose, 0);

      // Get transform for second camera (right)
      if (render_stereo) {

          geometry_msgs::TransformStamped camRightTransform;

          try{
              camRightTransform = tfBuffer_.lookupTransform("world", "uav/camera/right/internal_nwu", camLeftTransform.header.stamp);
          } catch (tf2::TransformException &ex) {
              ROS_WARN("Could NOT find transform for /uav/camera/right/nwu: %s", ex.what());
          }

          Transform3 camRightPose = tf2::transformToEigen(camRightTransform);
          flightGoggles.setCameraPoseUsingROSCoordinates(camRightPose, 1);
      }

      // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
      flightGoggles.state.ntime = camLeftTransform.header.stamp.toNSec();

      bIamgeReceived_ = false;

      // request render
      flightGoggles.requestRender();

      // reset scene reinitialize
      flightGoggles.state.scene.reinitialize &= false;
      scene_params_.reinitialize &= false;

      // Wait until image ready (polling every 1 us, timeout to 5ms)
      int cnt = 0;
      while(!bIamgeReceived_) {
        usleep(1);
        if (cnt == 5000) {
          // Fail responce
          response.result = false;
          return false;
        }
        ++cnt;
      }
    }

    // Success responce
    response.result = true;
    return true;
  }

  // Fail responce
  response.result = false;
  return false;
}

// New thread for republishing received images
void imageConsumer(ROSClient *self){
    while (true){
      if (self->b_have_renders_)
      {
        // Wait for render result (blocking).
        unity_incoming::RenderOutput_t renderOutput = self->flightGoggles.handleImageResponse();

        // Extract image timestamp
        ros::Time imageTimestamp;
//        imageTimestamp = imageTimestamp.fromNSec(renderOutput.renderMetadata.ntime);
        imageTimestamp = imageTimestamp.fromNSec(
                           renderOutput.renderMetadata.ntime
                           + self->sim_parameters_.cam_time_delay_ns());

        // Calculate average FPS every second.
	    std_msgs::Float32 fps_;
	    if (self->timeSinceLastMeasure_.toSec() != 0) {
	    double t = (ros::WallTime::now() - self->timeSinceLastMeasure_).toSec();
	        if (t > 1) {
	            fps_.data = self->frameCount_ / t;
	            self->fpsPublisher_.publish(fps_);
                self->timeSinceLastMeasure_ = ros::WallTime::now();
                self->frameCount_ = 0;
            }
        } else {
            self->timeSinceLastMeasure_ = ros::WallTime::now();
        }

        // Convert OpenCV image to image message
//        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", renderOutput.images[0]).toImageMsg();
        sensor_msgs::ImagePtr msg;
        self->imageDistortColor(msg, renderOutput.images[0]);
        msg->header.stamp = imageTimestamp;
        msg->header.frame_id = "uav/camera/left";
        // Add Camera info message for camera
        sensor_msgs::CameraInfoPtr cameraInfoMsgCopy(new sensor_msgs::CameraInfo(self->cameraInfoLeft));
        cameraInfoMsgCopy->header.frame_id = "uav/camera/left";
	cameraInfoMsgCopy->header.stamp = imageTimestamp;
	    self->imagePubLeft_.publish(msg, cameraInfoMsgCopy);

	    if (self->render_stereo) {
//            sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
//                                                                 renderOutput.images[1]).toImageMsg();
            sensor_msgs::ImagePtr msg_right;
            self->imageDistortColor(msg_right, renderOutput.images[1]);
            msg_right->header.stamp = imageTimestamp;
            msg_right->header.frame_id = "uav/camera/right";
            // Add Camera info message for camera
            sensor_msgs::CameraInfoPtr cameraInfoMsgCopy_Right(new sensor_msgs::CameraInfo(self->cameraInfoRight));
            cameraInfoMsgCopy_Right->header.frame_id = "uav/camera/right";
            cameraInfoMsgCopy_Right->header.stamp = imageTimestamp;
            self->imagePubRight_.publish(msg_right, cameraInfoMsgCopy_Right);
        }

        // Check for camera collision
        if (renderOutput.renderMetadata.hasCameraCollision){
            std_msgs::Empty msg;
            self->collisionPub_.publish(msg);
        }

        // Publish lidar range finder message
        sensor_msgs::Range lidarReturnMsg;
        lidarReturnMsg.header.stamp = imageTimestamp;
        lidarReturnMsg.header.frame_id = "uav/imu";
        lidarReturnMsg.radiation_type = lidarReturnMsg.INFRARED;
        lidarReturnMsg.field_of_view = 0;
        lidarReturnMsg.min_range = -1.0f*self->lidarMaxRange_;
        lidarReturnMsg.max_range = 0;
        // Add noise to lidar reading if reading is valid.
        // Make reading negative since the distance is in the -Z direction in '/uav/imu' frame.
        lidarReturnMsg.range = static_cast<float>(-1.0f * (renderOutput.renderMetadata.lidarReturn + sqrt(self->lidarVariance_) * self->standardNormalDistribution_(self->randomNumberGenerator_)));

        self->lidarPub_.publish(lidarReturnMsg);

        // Publish IR marker locations visible from FlightGoggles.
        flightgoggles::IRMarkerArray visiblePoints;
        visiblePoints.header.stamp = imageTimestamp;

        for (const unity_incoming::Landmark_t landmark : renderOutput.renderMetadata.landmarksInView){

            // Create marker msg
            flightgoggles::IRMarker marker;
            // Get name of gate and marker
            std::vector<std::string> IDTokens;
            boost::split(IDTokens, landmark.ID, boost::is_any_of("_"));

            marker.landmarkID.data = IDTokens.at(0);
            marker.markerID.data = IDTokens.at(1);

            // Get location and convert to pixel space with origin at upper left corner (OpenCV convention)
            marker.x = landmark.position.at(0) * renderOutput.renderMetadata.camWidth;
            // Flip pixel convention to match openCV. Origin is upper left after conversion.
            marker.y = (1.0f-landmark.position.at(1)) * renderOutput.renderMetadata.camHeight;
            // marker.z = landmark.position.at(2);
            marker.z = 0; // Do not publish distance.

            visiblePoints.markers.push_back(marker);
        }

        // Publish marker message
        self->irMarkerPub_.publish(visiblePoints);

        // Display result
        if (SHOW_DEBUG_IMAGE_FEED){
            cv::imshow("Debug RGB", renderOutput.images[0]);
            //cv::imshow("Debug D", renderOutput.images[1]);
            cv::waitKey(1);
        }

        self->frameCount_ ++;
        self->bIamgeReceived_ = true;
      }
      else
      {
        // sleep for 100ms in case now params were published yet
        ros::Duration(0.1).sleep();
      }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "flightgoggles_ros_bridge");

    ros::NodeHandle ns;
    ros::NodeHandle ns_private("flightgoggles_ros_bridge");

    // Create client
    ROSClient client(ns, ns_private);

    // Fork a sample image consumer thread
    std::thread imageConsumerThread(imageConsumer, &client);

    // Start render requests

//    callback tfCallback = boost::bind(&ROSClient::renderLoopTimerCallback, &client, _1);
//    ros::Subscriber sub = ns.subscribe("/tf", 1, tfSubscriber);

    // Spin
    ros::spin();

    return 0;
}

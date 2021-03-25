/**
 *******************************************************************************
 *
 * Copyright (C) 2018 Winter Guerra.
 * Copyright (C) 2020 Martin Scheiber, Control of Networked Systems,
 * University of Klagenfurt, Austria.
 *
 * All rights reserved.
 *
 * This software is licensed under the terms of the BSD-2-Clause-License with
 * the "Commons Clause" License Condition v1.0 which allows for non-commercial
 * use only, the full terms of which are made available in the LICENSE file.
 * No license in patents is granted.
 *
 * You can contact the author at martin.scheiber@aau.at
 *
 *******************************************************************************
 */

#ifndef UNITYMESSAGESPEC_H
#define UNITYMESSAGESPEC_H

// Message/state struct definition

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "json.hpp"
using json = nlohmann::json;

namespace unity_outgoing
{

struct Camera_t
{
  std::string ID;
  // Position and rotation use Unity left-handed coordinates.
  // Z North, X East, Y up.
  // E.G. East, Up, North.
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  int channels;
  bool isDepth;
  int outputIndex;
  // Should this camera collision check or check for visibility?
  bool hasCollisionCheck = true;
  bool doesLandmarkVisCheck = false;
};

// Window class for decoding the ZMQ messages.
struct Object_t
{
  std::string ID;
  std::string prefabID;
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  std::vector<double> size;
};

// Scene class for deconding the ZMQ messages.
struct Scene_t
{
  bool reinitialize;

  float objectsDensityMultiplier;
  float objectsDistributionScale;
  float objectsPlacementProbability;
  float objectsDensity;
  float objectsPlacement;
  float objectsPlacementMinFactor;
  float objectsPlacementMaxFactor;
  float objectsDistUniformity;
  float objectsDistOffset;
  float objectsDistMultiplier;
  float objectOriginRadius;

  int heightMeshFactor;
  float heightDistributionScale;
  float heightDistUniformity;
  float heightDistOffset;
  float heightDistMultiplier;
};


struct StateMessage_t
{
  // Scene/Render settings
  bool sceneIsInternal = true;
  // Scene choices in v1.4.1
  // std::string sceneFilename = "Hazelwood_Loft_Full_Night";
  // std::string sceneFilename = "Hazelwood_Loft_Full_Day";
  // std::string sceneFilename = "Butterfly_World";
  // std::string sceneFilename = "NYC_Subway";
  // std::string sceneFilename = "Museum_Day";
  std::string sceneFilename = "Museum_Day_Small";

  // Scene Metadata
  float illumination = 1.0f;
  float objectClutter = 0.5f;
  float objectFeatures = 0.5f;

  // Frame Metadata
  int64_t ntime;
  int camWidth = 1024;
  int camHeight = 768;
  float camFOV = 70.0f;
  double camDepthScale = 0.20; // 0.xx corresponds to xx cm resolution
  float camMotionBlur = 0.0f;

  // Object state update
  Scene_t scene;
  std::vector<Camera_t> cameras;
  std::vector<Object_t> objects;
//  std::vector<Landmark_t> landmarks;
};

// Json constructors

// StateMessage_t
inline void to_json(json &j, const StateMessage_t &o)
{
  j = json{// Initializers
//           {"maxFramerate", o.maxFramerate},
           {"sceneIsInternal", o.sceneIsInternal},
           {"sceneFilename", o.sceneFilename},

           // Scene Metadata
           {"illumination", o.illumination},
           {"objectClutter", o.objectClutter},
           {"objectFeatures", o.objectFeatures},

           // Frame Metadata
           {"ntime", o.ntime},
           {"camWidth", o.camWidth},
           {"camHeight", o.camHeight},
           {"camFOV", o.camFOV},
           {"camDepthScale", o.camDepthScale},
           {"camMotionBlur", o.camMotionBlur},
           // Object state update
           {"scene", o.scene},
           {"cameras", o.cameras},
           {"objects", o.objects}
  };
}

// Camera_t
inline void to_json(json &j, const Camera_t &o)
{
  j = json{{"ID", o.ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"channels", o.channels},
           {"isDepth", o.isDepth},
           {"outputIndex", o.outputIndex},
           {"hasCollisionCheck", o.hasCollisionCheck},
           {"doesLandmarkVisCheck", o.doesLandmarkVisCheck}
  };
}

// Object_t
inline void to_json(json &j, const Object_t &o)
  {
    j = json{
      {"ID", o.ID},
      {"prefabID", o.prefabID},
      {"position", o.position},
      {"rotation", o.rotation},
      {"size", o.size}
    };
  }

// Scene_t
inline void to_json(json &j, const Scene_t &o)
  {
    j = json{
      {"reinitialize", o.reinitialize},
      {"objectsDensityMultiplier", o.objectsDensityMultiplier},
      {"objectsDistributionScale", o.objectsDistributionScale},
      {"objectsPlacementProbability", o.objectsPlacementProbability},
      {"objectsDensity", o.objectsDensity},
      {"objectsPlacement", o.objectsPlacement},
      {"objectsPlacementMinFactor", o.objectsPlacementMinFactor},
      {"objectsPlacementMaxFactor", o.objectsPlacementMaxFactor},
      {"objectsDistUniformity", o.objectsDistUniformity},
      {"objectsDistOffset", o.objectsDistOffset},
      {"objectsDistMultiplier", o.objectsDistMultiplier},
      {"objectOriginRadius", o.objectOriginRadius},
      {"heightMeshFactor", o.heightMeshFactor},
      {"heightDistributionScale", o.heightDistributionScale},
      {"heightDistUniformity", o.heightDistUniformity},
      {"heightDistOffset", o.heightDistOffset},
      {"heightDistMultiplier", o.heightDistMultiplier}
    };
  }

}




// Struct for returning metadata from Unity.
namespace unity_incoming
{

    // Points in the world that should be checked for visibility from some camera.
    struct Landmark_t
    {
        std::string ID;
        std::vector<double> position;
    };

struct RenderMetadata_t
{
  // Metadata
  int64_t ntime;
  int camWidth;
  int camHeight;
  double camDepthScale;
  // Object state update
  std::vector<std::string> cameraIDs;
  std::vector<int> channels;

  // Status update from collision detectors and raycasters.
  bool hasCameraCollision;
  std::vector<Landmark_t> landmarksInView;
  float lidarReturn;
};

// Json Parsers

// Landmark_t
    inline void from_json(const json &j, Landmark_t &o)
    {
      o.ID = j.at("ID").get<std::string>();
      o.position = j.at("position").get<std::vector<double>>();
    }

// RenderMetadata_t
inline void from_json(const json &j, RenderMetadata_t &o)
{
  o.ntime = j.at("ntime").get<int64_t>();
  o.camWidth = j.at("camWidth").get<int>();
  o.camHeight = j.at("camHeight").get<int>();
  o.camDepthScale = j.at("camDepthScale").get<double>();
  o.cameraIDs = j.at("cameraIDs").get<std::vector<std::string>>();
  o.channels = j.at("channels").get<std::vector<int>>();
  o.hasCameraCollision = j.at("hasCameraCollision").get<bool>();
  o.landmarksInView = j.at("landmarksInView").get<std::vector<Landmark_t>>();
  o.lidarReturn = j.at("lidarReturn").get<float>();

}

// Struct for outputting parsed received messages to handler functions
struct RenderOutput_t
{
  RenderMetadata_t renderMetadata;
  std::vector<cv::Mat> images;
};
}

#endif

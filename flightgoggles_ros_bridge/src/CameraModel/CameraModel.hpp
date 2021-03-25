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

#ifndef CAMERAMODEL_H
#define CAMERAMODEL_H

// For image operations
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// For output operations
#include <iostream>
#include <string>

// For image displaying
#include <opencv2/highgui/highgui.hpp>

// Definitions for fast conversion
#define M_DEG2RAD 0.017453292519943295  //!< M_PI/180
#define M_RAD2DEG 57.29577951308232     //!< 180/M_PI

class CameraModel
{
  public:
    /// @name Lookup Struct
    struct LookupTable
    {
        cv::Mat x;
        cv::Mat y;
    };

    /// @name Con-/Destructors
    CameraModel(std::string _name="cam");
    ~CameraModel();

    /// @name Initial Calls
    virtual void  setCameraParameters(const cv::Point2i _resolution);
    virtual void  setCameraParameters(const uint _img_width, const uint _img_height);

    /// @name Image Modification
    virtual void  applyDistortion(cv::Mat &_image_in, cv::Mat &_image_out, bool _force_dim=false);
    virtual void  removeDistortion(cv::Mat &_image_in, cv::Mat &_image_out, bool _force_dim=false);
    virtual void  applyDistortion(cv::Mat &_image);
    virtual void  removeDistortion(cv::Mat &_image);

    /// @name Pixel Modification
    virtual cv::Point2f distortPixel(const cv::Point2f _P) const;
    virtual cv::Point2f undistortPixel(const cv::Point2f _pix) const;

    /// @name Getter Methods
    std::string         getCameraName() const;
    cv::Size2i          getCamResolution() const;
    cv::Size2i          getProjResolution() const;

    // Lookup Tables
    cv::Point2f         lookupDistortion(const int _row, const int _col) const;
    cv::Point2f         lookupUndistortion(const int _row, const int _col) const;

    /// @name Test Methods
    void                performTest(std::string _path, std::string _image_name="image.png");

  protected:
    /// @name Setter Methods
    void                setLookupDistortion(const cv::Mat &_lookup_x, const cv::Mat &_lookup_y);
    void                setLookupUndistortion(const cv::Mat &_lookup_x, const cv::Mat &_lookup_y);
    void                setCamResolution(const cv::Size2i _res);
    void                setProjResolution(const cv::Size2i _res);

  private:
    /// @name Camera Parameters
    std::string   name_;

    /// @name Image Parameters
    // Resolution
    cv::Size2i    cam_resolution_;          //!< describes the resolution of the physical, distorted image
    cv::Size2i    proj_resolution_;         //!< describes the resolution of the unprojected, undistorted image
    // Un/Distortion Correspondence
    LookupTable   lookup_distortion_;
    LookupTable   lookup_undistortion_;

};

#endif // CAMERAMODEL_H

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

#include "CameraModel.hpp"

/**
 * @brief CameraModel::CameraModel
 * @param _name
 */
CameraModel::CameraModel(std::string _name)
{
  name_ = _name;
} // CameraModel::CameraModel(...)

CameraModel::~CameraModel()
{

} // CameraModel::~CameraModel()

/**
 * @brief CameraModel::setCameraParameters sets the camera model parameters.
 * @param _img_width  width of the physical distorted image
 * @param _img_height height of the physical distorted image
 */
void CameraModel::setCameraParameters(
    const uint _img_width,
    const uint _img_height)
{
  cam_resolution_.width   = _img_width;
  cam_resolution_.height  = _img_height;
} // CameraModel::setCameraParameters(...)

/**
 * @brief CameraModel::setCameraParameters sets the camera model parameters.
 * @param _resolution cv::Point2i desired resolution of the physcical distorted image
 */
void CameraModel::setCameraParameters(const cv::Point2i _resolution)
{
  cam_resolution_.width   = _resolution.x;
  cam_resolution_.height  = _resolution.y;
} // CameraModel::setCameraParameters(...)

void CameraModel::applyDistortion(cv::Mat &_image)
{
  /// @todo
}

void CameraModel::applyDistortion(
    cv::Mat &_image_in,
    cv::Mat &_image_out,
    bool _force_dim)
{
  /// @todo
}

void CameraModel::removeDistortion(cv::Mat &_image)
{
  /// @todo
}

void CameraModel::removeDistortion(
    cv::Mat &_image_in,
    cv::Mat &_image_out,
    bool _force_dim)
{
  /// @todo
}

cv::Point2f CameraModel::distortPixel(const cv::Point2f _P) const
{
  // no distortion
  return _P;
}

cv::Point2f CameraModel::undistortPixel(const cv::Point2f _pix) const
{
  // no distortion
  return _pix;
}

// GETTER METHODS

std::string CameraModel::getCameraName() const
{
  return name_;
}

cv::Size2i CameraModel::getCamResolution() const
{
  return cam_resolution_;
}

cv::Size2i CameraModel::getProjResolution() const
{
  return proj_resolution_;
}

cv::Point2f CameraModel::lookupDistortion(
    const int _row,
    const int _col) const
{
  /// @todo check if row and col in matrix
  ///

//  std::cout << "Lookup size:    " << lookup_distortion_.x.size() << "\n"
//            << "       row,col: " << _row << ", " << _col << std::endl;

  return cv::Point2f(lookup_distortion_.x.at<float>(_row, _col),
                     lookup_distortion_.y.at<float>(_row, _col));
}

cv::Point2f CameraModel::lookupUndistortion(
    const int _row,
    const int _col) const
{
  return cv::Point2f(lookup_undistortion_.x.at<float>(_row, _col),
                     lookup_undistortion_.y.at<float>(_row, _col));
}

// SETTER METHODS

void CameraModel::setLookupDistortion(
    const cv::Mat &_lookup_x,
    const cv::Mat &_lookup_y)
{
  lookup_distortion_.x = _lookup_x.clone();
  lookup_distortion_.y = _lookup_y.clone();

//  std::cout << "Set x: "<< lookup_distortion_.x << std::endl;
//  std::cout << "Set y: "<< lookup_distortion_.y << std::endl;
}

void CameraModel::setLookupUndistortion(
    const cv::Mat &_lookup_x,
    const cv::Mat &_lookup_y)
{
  lookup_undistortion_.x = _lookup_x.clone();
  lookup_undistortion_.y = _lookup_y.clone();
}

void CameraModel::setCamResolution(const cv::Size2i _res)
{
  cam_resolution_ = _res;
}

void CameraModel::setProjResolution(const cv::Size2i _res)
{
  proj_resolution_ = _res;
}

// TEST METHODS

void CameraModel::performTest(
    std::string _path,
    std::string _image_name)
{
  cv::namedWindow("ImageWindow", cv::WINDOW_AUTOSIZE);

  std::string image_path = _path + "/" + _image_name;
  std::cout << "Performing test for:\n" << image_path << std::endl;
  cv::Mat in_img = cv::imread(image_path);

  if (!in_img.data)
  {
    std::cout << "Could not read image data." << std::endl;
    return;
  }

  cv::imshow("ImageWindow", in_img);

  cv::Mat dist_img;
  std::cout << "  Distorting image..." << std::endl;
  applyDistortion(in_img, dist_img);
  std::cout << "  ...done with " << dist_img.size().width << ", " << dist_img.size().height << std::endl;

  if (!dist_img.data)
  {
    std::cout << "Could not distort image data" << std::endl;
    return;
  }

  cv::imshow("ImageWindow", dist_img);
  cv::waitKey(0);

  // close opencv variables
  cv::destroyWindow("ImageWindow");
  in_img.release();
  dist_img.release();
}

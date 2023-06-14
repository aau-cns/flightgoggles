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

#include "FisheyeModel.hpp"

FisheyeCamera::FisheyeCamera()
{

} // FisheyeCamera::FisheyeCamera()

FisheyeCamera::~FisheyeCamera()
{

} // FisheyeCamera::~FisheyeCamera()

/**
 * @brief FisheyeCamera::setCameraParameters
 * @param _parameters
 */
void FisheyeCamera::setCameraParameters(const FisheyeParameters _parameters)
{
  /// @todo(martin): this
  updateLookups();
} // FisheyeCamera::setCameraParameters(...)

/**
 * @brief FisheyeCamera::setCameraParameters
 * @param _img_width
 * @param _img_height
 * @param fx
 * @param fy
 * @param cx
 * @param cy
 * @param s
 */
void FisheyeCamera::setCameraParameters(
    const uint _img_width,
    const uint _img_height,
    const float _fx,
    const float _fy,
    const float _cx,
    const float _cy,
    const float _s)
{

  cam_parameters_.img_width = _img_width;
  cam_parameters_.img_height = _img_height;
  cam_parameters_.fx = _fx;
  cam_parameters_.fy = _fy;
  cam_parameters_.cx = _cx;
  cam_parameters_.cy = _cy;
  cam_parameters_.s = _s;

  //  std::cout << "res: " << _img_width << ", " << _img_height << "\n"
  //            << "f:   " << _fx << ", " << _fy << "\n"
  //            << "c:   " << _cx << ", " << _cy << "\n"
  //            << "s:   " << _s << std::endl;

  cam_parameters_.setAdditional();

  updateLookups();
} // FisheyeCamera::setCameraParameters(...)

/**
 * @brief FisheyeCamera::updateLookups updates the distortion lookup tables.
 */
void FisheyeCamera::updateLookups()
{
  // get distortet (param) and undistorted (unproj) sizes
  cv::Size2i size_dis(cam_parameters_.img_width, cam_parameters_.img_height);
  cv::Size2i size_undis;

  cv::Point2f cns_ud[4];
  cns_ud[0] = undistortPixel(cv::Point2f(0.0, 0.0));
  cns_ud[1] = undistortPixel(cv::Point2f(size_dis.width, 0.0));
  cns_ud[2] = undistortPixel(cv::Point2f(size_dis.width, size_dis.height));
  cns_ud[3] = undistortPixel(cv::Point2f(0.0, size_dis.height));
  //  for (uint i = 0; i < 4; ++i) {
  //    std::cout << "    Corner " << i << ": " << cns_ud[i].x << ", " << cns_ud[i].y << std::endl;
  //  }

  size_undis = cv::Size2i(
      std::abs(std::round(std::fmax(cns_ud[1].x, cns_ud[2].x))) + std::abs(std::round(std::fmin(cns_ud[0].x, cns_ud[3].x))),
      std::abs(std::round(std::fmax(cns_ud[2].y, cns_ud[3].y))) + std::abs(std::round(std::fmin(cns_ud[0].y, cns_ud[1].y))));
  //  std::cout << "  Image resolutions:\n"
  //            << "    Dis:   " << size_dis.width << ", " << size_dis.height << "\n"
  //            << "    Undis: " << size_undis.width << ", " << size_undis.height << std::endl;

  // set resolutions
  setCamResolution(size_dis);
  setProjResolution(size_undis);

  // create distortion lookup
  // table used with undistorted->distorted image
  cv::Mat lookup_dis_x(size_undis, CV_32FC1);
  cv::Mat lookup_dis_y(size_undis, CV_32FC1);
  for (int col = 0; col < size_undis.width; ++col)
  {
    for (int row = 0; row < size_undis.height; ++row)
    {
      cv::Point2f pix_ud = undistortPixel(cv::Point2f(col, row));
      lookup_dis_x.at<float>(row, col) = pix_ud.x;
      lookup_dis_y.at<float>(row, col) = pix_ud.y;
    } // for (int row = 0; row < size_undis.height; ++row)
  }   // for (int col = 0; col < size_undis.width; ++col)
  setLookupDistortion(lookup_dis_x, lookup_dis_y);

  // create undistortion lookup
  // table used with distorted->undistorted image
  cv::Mat lookup_und_x(size_dis, CV_32FC1);
  cv::Mat lookup_und_y(size_dis, CV_32FC1);
  for (int col = 0; col < size_dis.width; ++col)
  {
    for (int row = 0; row < size_dis.height; ++row)
    {
      cv::Point2f pix_d = distortPixel(cv::Point2f(col, row));
      lookup_und_x.at<float>(row, col) = pix_d.x;
      lookup_und_y.at<float>(row, col) = pix_d.y;
    } // for (int row = 0; row < size_undis.height; ++row)
  }   // for (int col = 0; col < size_undis.width; ++col)
  setLookupUndistortion(lookup_und_x, lookup_und_y);

} // FisheyeCamera::updateLookups()

/**
 * @brief FisheyeCamera::applyDistortion applies the fisheye distortion to an image and keeps its dimensions.
 * @param _image cv::Mat image to distort
 */
void FisheyeCamera::applyDistortion(cv::Mat &_image)
{
  /// @todo
  cv::Mat out;
  applyDistortion(_image, out, true);
  _image.release();
  _image = out.clone();
  out.release();
} // FisheyeCamera::applyDistortion(...)

/**
 * @brief FisheyeCamera::applyDistortion applies the fisheye distortion to an image.
 * @param _image_in         input image to distort
 * @param _image_out        output image with distortion
 * @param _force_dim=false  keeps the dimension of the input image for the output image
 *
 * This applies the fisheye distortion to the image.
 * If _force_dim is not set, the output image is reduced to fit the distorted corners of the input image.
 */
void FisheyeCamera::applyDistortion(
    cv::Mat &_image_in,
    cv::Mat &_image_out,
    bool _force_dim)
{
  cv::Size proj_res = getProjResolution();
  cv::Size in_res = _image_in.size(); // should be equal getProjResolution()
  cv::Size out_res = getCamResolution();
  int in_channels = _image_in.channels();

  cv::Size in_res_half = getCamResolution() / 2;
  uint x_min = 0, y_min = 0;

  /// @todo perform check on res in which should be smaller than res out
  /// also perorm a check on the channels
  if (_force_dim || out_res.width > in_res.width || out_res.height > in_res.height)
    out_res = in_res;
  else if (in_res.width < proj_res.width || in_res.height < proj_res.height)
  {
    // create resolution based on available image
    std::cout << "[WARN] resolution change due to distortion" << std::endl;
    /// @todo create function that takes function pointer to derive dimension
    cv::Point2f cns_d[4];
    cns_d[0] = distortPixel(cv::Point2f(-in_res_half.width, -in_res_half.height));
    cns_d[1] = distortPixel(cv::Point2f(in_res_half.width, -in_res_half.height));
    cns_d[2] = distortPixel(cv::Point2f(in_res_half.width, in_res_half.height));
    cns_d[3] = distortPixel(cv::Point2f(-in_res_half.width, in_res_half.height));

    x_min = std::abs(std::round(std::fmin(cns_d[0].x, cns_d[3].x)));
    y_min = std::abs(std::round(std::fmin(cns_d[0].y, cns_d[1].y)));
    out_res = cv::Size2i(
        std::abs(std::round(std::fmax(cns_d[1].x, cns_d[2].x))) - x_min,
        std::abs(std::round(std::fmax(cns_d[2].y, cns_d[3].y))) - y_min);
  }

  cv::Mat img_out;
  if (_image_in.channels() == 1)
  {
    img_out = cv::Mat(out_res, CV_8UC1, cv::Scalar(0));

    // std::cout << "distorting output" << std::endl;
    for (uint col = 0; col < out_res.width; ++col)
    {
      for (uint row = 0; row < out_res.height; ++row)
      {
        // calculate the exact pixel location
        cv::Point2f pix_ud = lookupDistortion(row + y_min, col + x_min);
        cv::Point2i pix_ref(std::floor(pix_ud.x), std::floor(pix_ud.y));
        cv::Point2f pix_diff = pix_ud - cv::Point2f(pix_ref.x, pix_ref.y);

        // calculate the color composition
        int ref_w = pix_ref.x + in_res_half.width;
        int ref_h = pix_ref.y + in_res_half.height;

        // check if pixel is within dimensions
        if (ref_w >= 0 &&
            ref_w < (int)in_res.width - 1 &&
            ref_h >= 0 &&
            ref_h < (int)in_res.height - 1)
        {
          img_out.at<uchar>(row, col) =
              pix_diff.x * (pix_diff.y * _image_in.at<uchar>(ref_h + 1, ref_w + 1) + (1 - pix_diff.y) * _image_in.at<uchar>(ref_h, ref_w + 1)) + (1 - pix_diff.x) * (pix_diff.y * _image_in.at<uchar>(ref_h + 1, ref_w) + (1 - pix_diff.y) * _image_in.at<uchar>(ref_h, ref_w));
        } // if
        else
        {
          img_out.at<uchar>(row, col) = (uchar)0;
        } // else
      }   // for (uint col = 0; col < res.x; ++col)
    }     // for (uint row = 0; row < res.y; ++row)

  } // _image_in.channels()
  else if (in_channels == 3)
  {
    img_out = cv::Mat(out_res, CV_8UC3, cv::Scalar(0));

    // std::cout << "distorting output" << std::endl;
    for (uint col = 0; col < out_res.width; ++col)
    {
      for (uint row = 0; row < out_res.height; ++row)
      {
        // calculate the exact pixel location
        cv::Point2f pix_ud = lookupDistortion(row + y_min, col + x_min);
        cv::Point2i pix_ref(std::floor(pix_ud.x), std::floor(pix_ud.y));
        cv::Point2f pix_diff = pix_ud - cv::Point2f(pix_ref.x, pix_ref.y);

        // calculate the color composition
        int ref_w = pix_ref.x + in_res_half.width;
        int ref_h = pix_ref.y + in_res_half.height;

        // check if pixel is within dimensions
        if (ref_w >= 0 &&
            ref_w < (int)in_res.width - 1 &&
            ref_h >= 0 &&
            ref_h < (int)in_res.height - 1)
        {
          img_out.at<cv::Vec3b>(row, col) =
              pix_diff.x * (pix_diff.y * _image_in.at<cv::Vec3b>(ref_h + 1, ref_w + 1) + (1 - pix_diff.y) * _image_in.at<cv::Vec3b>(ref_h, ref_w + 1)) + (1 - pix_diff.x) * (pix_diff.y * _image_in.at<cv::Vec3b>(ref_h + 1, ref_w) + (1 - pix_diff.y) * _image_in.at<cv::Vec3b>(ref_h, ref_w));
        } // if
        else
        {
          img_out.at<cv::Vec3b>(row, col) = 0, 0, 0;
        } // else
      }   // for (uint col = 0; col < res.x; ++col)
    }     // for (uint row = 0; row < res.y; ++row)
  }
  else
  {
    std::cout << "[WARN] Image channels type not known" << std::endl;
    return;
  }

  // clone image to output
  _image_out = img_out.clone();
  img_out.release();

} // FisheyeCamera::applyDistortion(...)

void FisheyeCamera::removeDistortion(cv::Mat &_image)
{
  /// @todo
}

void FisheyeCamera::removeDistortion(
    cv::Mat &_image_in,
    cv::Mat &_image_out,
    bool _force_dim)
{
  /// @todo
} // FisheyeCamera::removeDistortion(...)

/**
 * @brief FisheyeCamera::distortPixel
 * @param _P
 * @return
 *
 * Distorts a pixel from the image plane (z1) to the cam plane.
 * Uses the given parameters to calculate and apply the distortion.
 *
 * @cite Straight Lines Have to be Straight
 */
cv::Point2f FisheyeCamera::distortPixel(const cv::Point2f _P) const
{
  float P_x = _P.x / cam_parameters_.fx;
  float P_y = _P.y / cam_parameters_.fy;

  float P_r = std::sqrt(P_x * P_x + P_y * P_y);
  float ud_factor = 1.0;
  if (P_r > 0.001)
  {
    if (cam_parameters_.s == 0)
      ud_factor = P_r / P_r;
    else
      ud_factor = (cam_parameters_.s_inv * std::atan(cam_parameters_.s_2tan * P_r)) / P_r;
  }

  float px_nx = ud_factor * P_x;
  float px_ny = ud_factor * P_y;

  float px = cam_parameters_.cx + cam_parameters_.fx * px_nx;
  float py = cam_parameters_.cy + cam_parameters_.fy * px_ny;

  return cv::Point2f(px, py);
} // FisheyeCamera::distortPixel(...) const

/**
 * @brief FisheyeCamera::undistortPixel
 * @param _pix
 * @return
 *
 * Unprojects the pixel onto the image (z=1) plane using the fisheye distortion model.
 * Then uses the pinhole camera parameters (fx) to proide the pixel in the rectified, undistorted image.
 *
 * @cite Straight Lines Have to be Straight
 */
cv::Point2f FisheyeCamera::undistortPixel(const cv::Point2f _pix) const
{
  float px_nx = (_pix.x - cam_parameters_.cx) / cam_parameters_.fx;
  float px_ny = (_pix.y - cam_parameters_.cy) / cam_parameters_.fy;

  float px_r = std::sqrt(px_nx * px_nx + px_ny * px_ny);
  float d_factor = 1.0;
  if (px_r > 0.01)
  {
    if (cam_parameters_.s == 0)
      d_factor = px_r / px_r;
    else
      d_factor = (std::tan(px_r * cam_parameters_.s) * cam_parameters_.s_2tan_inv) / px_r;
  }

  float P_x = d_factor * px_nx * cam_parameters_.fx;
  float P_y = d_factor * px_ny * cam_parameters_.fy;

  return cv::Point2f(P_x, P_y);
} // FisheyeCamera::undistortPixel(...)

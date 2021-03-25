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

#ifndef FISHEYEMODEL_H
#define FISHEYEMODEL_H

#include "CameraModel.hpp"

class FisheyeCamera : public CameraModel
{
  public:
    /// @name Parameter Struct
    /**
     * @brief The FisheyeParameters struct describes the parameters used in the fisheye model.
     */
    struct FisheyeParameters
    {
        /// @name Parameters
        uint  img_width;    //!< resolution width
        uint  img_height;   //!< resolution height
        float fx;           //!< focal length in x
        float fy;           //!< focal length in y
        float cx;           //!< center point in x
        float cy;           //!< center point in y
        float s;            //!< fisheye distortion parameter

        /// @name Precalculated Parameters
        float s_inv;        //!< inverse fisheye distortion parameter
        float s_2tan;       //!< two times tan of s \f$ 2 * \tan(\frac{s}{2}) \f$
        float s_2tan_inv;   //!< inverse of two times the tan of s

        /**
         * @brief setAdditional sets additional calculated parameters
         */
        void setAdditional() {
          if (! s == 0)
          {
            s_inv = 1/s;
            s_2tan = 2.0 * std::tan(s / 2.0);
            s_2tan_inv = 1.0 / (s_2tan);
          } // if
        } // setAdditional()
    }; // struct FisheyeParameters

    /// @name Con-/Destructors
    FisheyeCamera();
    ~FisheyeCamera();

    /// @name Initial Calls
    virtual void  setCameraParameters(const FisheyeParameters _parameters);
    virtual void  setCameraParameters(const uint _img_width, const uint _img_height, const float _fx, const float _fy,
                                      const float _cx, const float _cy, const float _s);

    /// @name Image Modification
    virtual void  applyDistortion(cv::Mat &_image_in, cv::Mat &_image_out, bool _force_dim=false);
    virtual void  removeDistortion(cv::Mat &_image_in, cv::Mat &_image_out, bool _force_dim=false);
    virtual void  applyDistortion(cv::Mat &_image);
    virtual void  removeDistortion(cv::Mat &_image);

    /// @name Pixel Modification
    virtual cv::Point2f distortPixel(const cv::Point2f _P) const;
    virtual cv::Point2f undistortPixel(const cv::Point2f _pix) const;

  protected:
  private:
    /// @name Model Parameters
    // Static variables
    FisheyeParameters cam_parameters_;

    // Methods
    void          updateLookups();
}; // class FisheyeCamera

#endif

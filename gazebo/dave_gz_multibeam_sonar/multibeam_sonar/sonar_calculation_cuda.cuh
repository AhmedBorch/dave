/*
 * Copyright 2020 Naval Postgraduate School
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once
#include <cuda.h>
#include <thrust/complex.h>
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include "device_launch_parameters.h"

#include <stdio.h>
#include <complex>
#include <iostream>
#include <valarray>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>

namespace NpsGazeboSonar
{

typedef std::complex<float> Complex;
typedef std::valarray<Complex> CArray;
typedef std::valarray<CArray> CArray2D;
typedef std::valarray<CArray2D> CArray3D;

/// \brief CUDA Device Check Function Wrapper
void check_cuda_init_wrapper(void);

/// \brief CUDA Free Memory Function
void free_cuda_memory();

/// \brief Sonar Claculation Function Wrapper
CArray2D sonar_calculation_wrapper(
  const cv::Mat & depth_image, const cv::Mat & normal_image, double _hPixelSize, double _vPixelSize,
  double _hFOV, double _vFOV, double _beam_azimuthAngleWidth, double _beam_elevationAngleWidth,
  double _ray_azimuthAngleWidth, float * _ray_elevationAngles, double _ray_elevationAngleWidth,
  double _soundSpeed, double _maxDistance, double _sourceLevel, int _nBeams, int _nRays,
  int _raySkips, double _sonarFreq, double _bandwidth, int _nFreq,
  const cv::Mat & reflectivity_image, double _attenuation, float * _window, float ** _beamCorrector,
  float _beamCorrectorSum, bool _debugFlag, bool _blazingFlag);

/// \brief Sonar 3D Volumetric Calculation Function Wrapper
/// \param _nBeams_v Vertical beam count for 3D frustum
/// \param _vFOV Vertical Field of View
CArray3D sonar_calculation_3d_wrapper(
  const cv::Mat & depth_image, const cv::Mat & normal_image, double _hPixelSize, double _vPixelSize,
  double _hFOV, double _vFOV, double _beam_azimuthAngleWidth, double _beam_elevationAngleWidth,
  double _ray_azimuthAngleWidth, float * _ray_elevationAngles, double _ray_elevationAngleWidth,
  double _soundSpeed, double _maxDistance, double _sourceLevel, 
  int _nBeams_h, int _nBeams_v, // 3D dimensions
  int _nRays_h, int _nRays_v,
  int _raySkips, double _sonarFreq, double _bandwidth, int _nFreq,
  const cv::Mat & reflectivity_image, double _attenuation, float * window, 
  float ** beamCorrector, float beamCorrectorSum, bool debugFlag, bool blazingFlag);

}  // namespace NpsGazeboSonar
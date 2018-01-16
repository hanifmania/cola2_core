
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/SGDerivation.h"

SGDerivation::SGDerivation():
  _last_data(0.0),
  _is_buffer_init(false)
{
  // Init savitzky golay coeffs
  _savitzky_golay_coeffs.push_back(0.2);
  _savitzky_golay_coeffs.push_back(0.1);
  _savitzky_golay_coeffs.push_back(0.0);
  _savitzky_golay_coeffs.push_back(-0.1);
  _savitzky_golay_coeffs.push_back(-0.2);
}

double SGDerivation::addValue(const double data, double dt)
{
  if (dt <= 0.01 ) dt = 0.05;
  if (_is_buffer_init)
  {
    double inc = cola2::util::normalizeAngle(data - _last_data);
    _buffer.push_back(_last_data + inc);
    _buffer.pop_front();
    _last_data += inc;
    return convolve(_buffer, _savitzky_golay_coeffs) / dt;
  }
  else
  {
    if (_buffer.size() == 0)
    {
      // First element
      _buffer.push_back(data);
      _last_data = data;
    }
    else
    {
      double inc = cola2::util::normalizeAngle(data - _last_data);
      _buffer.push_back(_last_data + inc);

      // Check if the buffer is initialized
      if(_buffer.size() == _savitzky_golay_coeffs.size())
      {
        _is_buffer_init = true;
        return convolve(_buffer, _savitzky_golay_coeffs) / dt;
      }
    }
  }
  return 0.0;
}

double SGDerivation::convolve(const std::deque<double> data1, const std::vector<double> data2)
{
  assert(data1.size() == data2.size());
  double ret = 0.0;
  for (unsigned int i = 0; i < data1.size(); i++)
  {
    ret = ret + (data1.at(i) * data2.at(data2.size() - i - 1));
  }
  return ret;
}

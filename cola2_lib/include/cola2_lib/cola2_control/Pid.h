
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_PID_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_PID_H_

#include <map>
#include <string>

#include "IController.h"

class Pid: public IController
{
 public:
    Pid(std::string name);

    void reset();
    double compute(double time_in_sec, double setpoint, double feedback);
    bool setParameters(std::map<std::string, double> params);

 private:
    double _kp;
    double _ti;
    double _td;
    double _i_limit;
    double _fff;
    double _time_old;
    double _feedback_old;
    double _error_old;
    double _eik_old;
    bool _derivative_term_from_feedback;

    // for filtering purposes
    double _edotk_old;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_PID_H_

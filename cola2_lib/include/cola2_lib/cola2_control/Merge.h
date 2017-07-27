
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MERGE_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MERGE_H_

#include <algorithm>
#include <string>
#include <vector>

#include "./Request.h"

class Merge
{
 public:
    Merge(std::string name, std::string type, double expire_time);

    void addRequest(const Request req);
    Request merge(double current_time);

 private:
    std::string _name;
    std::string _requester;
    std::string _type;
    double _expire_time;
    std::vector< Request > _messages;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_MERGE_H_

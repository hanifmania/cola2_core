
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/cola2_control/Merge.h"

Merge::Merge(std::string name, std::string type, double expire_time):
  _name(name),
  _requester(name),
  _type(type),
  _expire_time(expire_time)
{
}

void Merge::addRequest(const Request req)
{
  // If there is a request from the same requester remove it
  std::vector< Request >::iterator it;
  it = std::find(_messages.begin(), _messages.end(), req);
  if (it != _messages.end())
  {
    // std::cout << "Erase previous req <" << it->getRequester() << "> from same requester\n";
    _messages.erase(it);
  }

  // Add request to messages list if not all axis are disables
  if (!req.isAllDisabled()) _messages.push_back(req);
}

Request Merge::merge(double current_time)
{
  // Check if some request has expired
  bool until_the_end = false;
  while (!until_the_end)
  {
    bool to_be_deleted = false;
    std::vector< Request >::iterator it = _messages.begin();
    while (it != _messages.end() && !to_be_deleted)
    {
      if ((current_time - it->getStamp()) > _expire_time)
      {
        to_be_deleted = true;
      }
      else
      {
        it++;
      }
    }
    if (to_be_deleted)
    {
      // std::cout << "Erase (time expire): " << it->getRequester() << "\n";
      _messages.erase(it);
    }
    else
    {
      until_the_end = true;
    }
  }

  // If no request to merge return empty Request
  if (_messages.size() == 0)
  {
    // std::cout << " Merged 0 \n";
    return Request(_requester, current_time);
  }

  // If only one request return it
  if (_messages.size() == 1)
  {
    // std::cout << " Merged 1 \n";
    return _messages.at(0);
  }

  // If more than one request merge them
  // Sort requests in reverse order
  std::sort(_messages.begin(), _messages.end());
  std::reverse(_messages.begin(), _messages.end());

  // Combine request with lower priorities
  Request merged_req(_messages.at(0));
  // std::cout << " Merged " << _messages.size() << "\n";
  // std::cout << "--> " << _messages.at(0).getRequester() << "\n";
  for (unsigned int i = 1; i < _messages.size(); i++)
  {
    merged_req.combineRequest(_messages.at(i), _type);
    // std::cout << "--> " << _messages.at(i).getRequester() << "\n";
  }
  return merged_req;
}

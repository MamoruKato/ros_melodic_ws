#ifndef _PROXIMITY_SERVER_H
#define _PROXIMITY_SERVER_H

#include "laser_scan_subscriber.h"
#include "testing_subs_pubs/laserScanData.h"

class ProximityServer
{
public:
  ProximityServer();
  ~ProximityServer();
  bool ProximityServerCallback(testing_subs_pubs::laserScanData::Request& req,
                               testing_subs_pubs::laserScanData::Response& res);
private:
    ros::ServiceServer _service;
    ros::NodeHandle _nh;
    LaserScanSubscriber _laserSub;

};


#endif

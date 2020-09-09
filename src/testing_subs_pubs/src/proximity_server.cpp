#include "includes/proximity_server.h"

ProximityServer::ProximityServer()
{
  _service = _nh.advertiseService("proximity_warning", &ProximityServer::ProximityServerCallback, this);
  return;
}

ProximityServer::~ProximityServer()
{
  return;
}

bool ProximityServer::ProximityServerCallback(testing_subs_pubs::laserScanData::Request& req,
                                              testing_subs_pubs::laserScanData::Response& res)
{
  res.value.data = _laserSub.getLaserScanData().ranges[70];
  if(_laserSub.getLaserScanData().ranges[70] < 0.5)
  {
    res.warning.data = "Closest Level of proximity Reached";
  }
  else
  {
    res.warning.data = "Level of Proximity: Safe";
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "proximity_warning_server");
  ProximityServer _server;
  ros::spin();

  return 0;
}

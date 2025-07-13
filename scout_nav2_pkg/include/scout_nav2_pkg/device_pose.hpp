#ifndef SCOUT_NAV2_PKG_DEVICE_POSE_HPP_
#define SCOUT_NAV2_PKG_DEVICE_POSE_HPP_

struct Point
{
  double x;
  double y;
  double z;
};

struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};

struct DevicePose
{
  int device_id;
  Point point;
  Quaternion quaternion;
};

#endif // SCOUT_NAV2_PKG_DEVICE_POSE_HPP_

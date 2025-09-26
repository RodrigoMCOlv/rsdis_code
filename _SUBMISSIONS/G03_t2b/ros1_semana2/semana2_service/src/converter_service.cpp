#include <ros/ros.h>
#include <cmath>

#include "semana2_service/D2R.h"
#include "semana2_service/R2D.h"
#include "semana2_service/G2ECEF.h"
#include "semana2_service/ECEF2G.h"

// deg -> rad
bool d2r_callback(semana2_service::D2R::Request &req,
                  semana2_service::D2R::Response &res)
{
  res.radians = (req.degrees * M_PI) / 180.0;
  return true;
}

// rad -> deg
bool r2d_callback(semana2_service::R2D::Request &req,
                  semana2_service::R2D::Response &res)
{
  res.degrees = (req.radians * 180.0) / M_PI;
  return true;
}

// geodetic -> ECEF
bool g2ecef_callback(semana2_service::G2ECEF::Request &req,
                     semana2_service::G2ECEF::Response &res)
{
  const double a  = 6378137.0;                    // WGS84 semi-major axis [m]
  const double e2 = 6.6943799901413165e-3;        // first ecc. squared

  const double lat = req.latitude  * M_PI / 180.0;
  const double lon = req.longitude * M_PI / 180.0;
  const double h   = req.altitude;

  const double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));

  res.x = (N + h) * std::cos(lat) * std::cos(lon);
  res.y = (N + h) * std::cos(lat) * std::sin(lon);
  res.z = (N * (1.0 - e2) + h) * std::sin(lat);
  return true;
}

// ECEF -> geodetic (iterative-free approximation)
bool ecef2g_callback(semana2_service::ECEF2G::Request &req,
                     semana2_service::ECEF2G::Response &res)
{
  const double X = req.x;
  const double Y = req.y;
  const double Z = req.z;

  const double a  = 6378137.0;
  const double b  = 6356752.314245179;
  const double e2 = 0.0066943799901413165;  // first ecc^2
  const double ep2= 0.006739496742276434;   // second ecc^2

  const double lon = std::atan2(Y, X);
  const double p = std::sqrt(X*X + Y*Y);
  const double theta = std::atan2(Z * a, p * b);
  const double sin_t = std::sin(theta), cos_t = std::cos(theta);

  const double lat = std::atan2(Z + ep2 * b * sin_t*sin_t*sin_t,
                                p - e2  * a * cos_t*cos_t*cos_t);

  const double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
  const double h = p / std::cos(lat) - N;

  res.latitude  = lat * 180.0 / M_PI;
  res.longitude = lon * 180.0 / M_PI;
  res.altitude  = h;
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ConverterNode");
  ros::NodeHandle nh;

  ros::ServiceServer s1 = nh.advertiseService("d2r",    d2r_callback);
  ros::ServiceServer s2 = nh.advertiseService("r2d",    r2d_callback);
  ros::ServiceServer s3 = nh.advertiseService("g2ecef", g2ecef_callback);
  ros::ServiceServer s4 = nh.advertiseService("ecef2g", ecef2g_callback);

  ROS_INFO("UGV Coordinate Converter Service is up.");
  ros::spin();
  return 0;
}

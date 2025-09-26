#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include "my_interfaces/srv/d2_r.hpp"
#include "my_interfaces/srv/r2_d.hpp"
#include "my_interfaces/srv/g2_ecef.hpp"
#include "my_interfaces/srv/ecef2_g.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ConverterNode : public rclcpp::Node {
public:
    ConverterNode() : Node("my_interfaces_node") {
        using namespace std::placeholders;

        d2r_service_ = this->create_service<my_interfaces::srv::D2R>(
            "d2r", std::bind(&ConverterNode::d2r_callback, this, _1, _2));
        r2d_service_ = this->create_service<my_interfaces::srv::R2D>(
            "r2d", std::bind(&ConverterNode::r2d_callback, this, _1, _2));
        g2ecef_service_ = this->create_service<my_interfaces::srv::G2ECEF>(
            "g2ecef", std::bind(&ConverterNode::g2ecef_callback, this, _1, _2));
        ecef2g_service_ = this->create_service<my_interfaces::srv::ECEF2G>(
            "ecef2g", std::bind(&ConverterNode::ecef2g_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "UGV Coordinate Converter Service is up.");
    }
private:
    rclcpp::Service<my_interfaces::srv::D2R>::SharedPtr d2r_service_;
    rclcpp::Service<my_interfaces::srv::R2D>::SharedPtr r2d_service_;
    rclcpp::Service<my_interfaces::srv::G2ECEF>::SharedPtr g2ecef_service_;
    rclcpp::Service<my_interfaces::srv::ECEF2G>::SharedPtr ecef2g_service_;
    void d2r_callback(
        const std::shared_ptr<my_interfaces::srv::D2R::Request> request,
        std::shared_ptr<my_interfaces::srv::D2R::Response> response) {

        response->radians = (request->degrees * M_PI)/180;
    }
 
    void r2d_callback(
        const std::shared_ptr<my_interfaces::srv::R2D::Request> request,
        std::shared_ptr<my_interfaces::srv::R2D::Response> response) {
        response->degrees = (request->radians*180)/M_PI;
    }

    void g2ecef_callback(
      const std::shared_ptr<my_interfaces::srv::G2ECEF::Request> request,
      std::shared_ptr<my_interfaces::srv::G2ECEF::Response> response) {

      const double a  = 6378137.0;                                   // semi-major axis [m]
      const double first_eccentricity = 6.6943799901413165e-3;       // first eccentricity squared

      const double lat = request->latitude  * M_PI / 180.0;
      const double lon = request->longitude * M_PI / 180.0;
      const double h   = request->altitude;

      const double N = a / sqrt(1.0 - first_eccentricity * sin(lat) * sin(lat));

      const double X = (N + h) * cos(lat) * cos(lon);
      const double Y = (N + h) * cos(lat) * sin(lon);
      const double Z = (N * (1.0 - first_eccentricity) + h) * sin(lat);

      response->x = X;
      response->y = Y;
      response->z = Z;
    }
    void ecef2g_callback(
        const std::shared_ptr<my_interfaces::srv::ECEF2G::Request> request,
        std::shared_ptr<my_interfaces::srv::ECEF2G::Response> response) {
            float X =    request -> x;
            float Y =    request -> y;
            float Z =    request -> z;


        float  longitude_rad = atan2(Y,X);
        
        float p = sqrt(pow(X,2) + pow(Y,2));
        float a = 6378137.0;
        float b = 6356752.314245179;
        float f=1/298.257223563;
        float first_eccentricity = 0.0066943799901413165;
        float second_eccentricity = 0.006739496742276434;
        float teta = atan((Z*a)/(p*b));
        float latitude_rad = atan((Z+second_eccentricity*b*pow(sin(teta),3))/(p-first_eccentricity*a*pow(cos(teta),3)));

        float N = a/(sqrt(1-first_eccentricity*pow(sin(latitude_rad),2)));
        
        float altitude  = (p/cos(latitude_rad)) - N;

        float latitude_deg = (latitude_rad*180)/M_PI;
        float longitude_deg = (longitude_rad*180)/M_PI;

        response -> latitude = latitude_deg;
        response -> longitude = longitude_deg;
        response -> altitude = altitude;
    }
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConverterNode>());
    rclcpp::shutdown();
    return 0;
}
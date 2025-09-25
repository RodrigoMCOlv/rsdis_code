//TO BUILD
//colcon build --symlink-install

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "my_interfaces/msg/chernobyl_sensors.hpp"
#include "rclcpp/rclcpp.hpp"
#include <random>

// Defina fora da função ou como membro da classe
std::random_device rd;   // gera semente aleatória
std::mt19937 gen(rd());  // motor Mersenne Twister
std::uniform_real_distribution<> dis(0, 1);
using namespace std::chrono_literals;


float tempSimulator();
float GPS_Simulator(int cord);
float Humidity_Simulator();

class Rover_Sensors : public rclcpp::Node{
  public:
    Rover_Sensors()
    : Node("rover_sensors")
    {
      
      publisher_ = this->create_publisher<my_interfaces::msg::ChernobylSensors>("rover_sensors_pub", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&Rover_Sensors::timer_callback, this));
    

    }

  private:
    void timer_callback() 
    {
      my_interfaces::msg::ChernobylSensors msg;
      
      
      msg.header.stamp = this->get_clock()->now();
      msg.temperature.temperature = tempSimulator();
      msg.gps.latitude=GPS_Simulator(0);
      msg.gps.longitude=GPS_Simulator(1);
      msg.gps.altitude=GPS_Simulator(2);
      msg.humidity.relative_humidity=Humidity_Simulator();
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing rover_sensors:\n  Time: %d \n Temperature: %.2f \n GPS->   lat: %f   |   long: %f   |   alt: %f \n Relative Humidity: %.2f %\n",
        msg.header.stamp.sec ,msg.temperature.temperature, msg.gps.latitude,msg.gps.longitude, msg.gps.altitude, msg.humidity.relative_humidity
      );

      publisher_->publish(msg);
    }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_interfaces::msg::ChernobylSensors>::SharedPtr publisher_;


};
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rover_Sensors>());
  rclcpp::shutdown();
  return 0;
  
}

float temp = 0;
float way;

float tempSimulator()
{
  way = dis(gen);
  
  if (way < 0.4) temp -= 0.5;
  else if (way > 0.5) temp += 0.5;
  
  
  if (temp < -20) temp = -20;
  if (temp > 20) temp = 20;


  return temp;
}



//51.270497, 30.245486

//51.280040, 30.193330
//Chernobyl GPS Coords
volatile float longitude = 51.275;
volatile float latitude  = 30.22;
volatile float altitude  = 0;


float GPS_Simulator(int cord){
  //0 = lat , 1 = long, 2 = alt

  way = dis(gen);

  if(cord == 0){

    if (way < 0.45) latitude -= 0.0001;
    else if (way > 0.55) latitude += 0.0001;
    
    if (latitude < 51.270497) latitude = 51.270497;
    if (latitude > 51.280040) latitude = 51.280040;
    
    return latitude;
  }
  
  if(cord == 1){
      if (way < 0.45) longitude -= 0.00001;
    else if (way > 0.55) longitude += 0.00001;
    
    if (longitude > 30.245486) longitude = 30.245486;
    if (longitude < 30.193330) longitude = 30.193330;

    return longitude;
  }
  

  if(cord == 2){
    if (way < 0.45) altitude -= 0.2;
    else if (way > 0.55) altitude += 0.2;
    
    if (altitude < 0) altitude = 0;
    if (altitude > 10) altitude = 10;

    return altitude;
  }

  return 0;
}
float relative_humidity=0;
float Humidity_Simulator(){
  //entre 20% e 75%
  way = dis(gen);
  if (way < 0.4) relative_humidity -= 0.05;
  else if (way > 0.5) relative_humidity += 0.05;
  
  if (relative_humidity < 0.20) relative_humidity = 0.20;
  if (relative_humidity > 0.75) relative_humidity = 0.75;
  return relative_humidity*100;
}
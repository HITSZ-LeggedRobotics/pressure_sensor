#include "ros/ros.h"
#include "pressure_sensor/pressuresensorrs485driver.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pressure_sensor_driver_demo_node");
  ros::NodeHandle node_handle("~");
  sri_driver::PressureSensorRS485Driver pressureSensor(node_handle, "/dev/ttyUSB0", 9600);
      while (ros::ok()) {
      pressureSensor.start();
  ros::spin();
    }

  return 0;
}

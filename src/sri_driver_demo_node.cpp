#include "ros/ros.h"
#include "force_sensor/sri3dforcesensorrs232driver.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sri_driver_demo_node");
  ros::NodeHandle node_handle("~");
  sri_driver::SRI3DForceSensorRS232Driver forceSensor(node_handle, "/dev/ttyUSB0", 115200);
  while (ros::ok()) {
      forceSensor.start();
      ros::spin();
    }

  return 0;
}


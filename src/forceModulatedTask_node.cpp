#include "ForceModulatedTask.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "motionController");
  ros::NodeHandle n;
  float frequency = 1000.0f;

  ForceModulatedTask fmt(n,frequency);
  return 0;
}

#include "ForceModulatedTask.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "forceModulatedTask");
  ros::NodeHandle n;
  float frequency = 1000.0f;

  ForceModulatedTask fmt(n,frequency);

  if (!fmt.init()) 
  {
    return -1;
  }
  else
  {
   
    fmt.run();
  }

  return 0;
}

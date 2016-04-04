#include <ros/ros.h>
#include "prfabmap.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "place_recognizer_main");
 
  PRfabmap pr_fabmap;
  ros::spin();
  
  
  /*while(ros::ok())
  {
    
    ROS_INFO("Publishing Matching Info (Dummy)");
    pr_fabmap.publishMatchInfoDebug();
    ros::spinOnce();
  }
 */
  
  return 0;
}

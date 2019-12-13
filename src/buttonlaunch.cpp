#include <iostream>
#include <ros/ros.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <std_msgs/String.h>

void buttonCallback(const kobuki_msgs::ButtonEvent &msg)
{
  bool conditionI = false;

  int buttonstate1 = 0;

  int butttonstate2 = 0;

  buttonstate1 = msg.Button1;

  butttonstate2 = msg.Button2;

  // denne funktion vil starte launchfilen for linefollower

  // if(butttonstate2==2

  //   system("rosnode kill velocity" "rosnode kill detection" "rosnode kill dockdetector");

  // }

  if (conditionI && buttonstate1 == 1)
  {
    if (conditionI == false)
    {
      std::cout << "The button is on" << std::endl;

      system("roslaunch final_pkg larcus.launch");

      system("roslaunch final_pkg lf.launch");

      system("rostopic echo /odom");

      conditionI = true;
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "buttonlaunch");

  ros::NodeHandle n;

  ros::Subscriber recall_sub = n.subscribe("/mobile_base/events/button", 5, buttonCallback);

  kobuki_msgs::ButtonEvent msg;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

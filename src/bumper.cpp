#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Led.h>

#include <iostream>
#include <string>

kobuki_msgs::Led LEDmsg;
kobuki_msgs::BumperEvent bumpMsg;
ros::Publisher cmd_vel_pub;
ros::Publisher LED1_pub;
ros::Publisher LED2_pub;
ros::Subscriber bumperSub;

void bump(const kobuki_msgs::BumperEvent &bumpMsg)
{
    if (bumpMsg.state == 1)
    {
        LEDmsg.value = 3;
        LED1_pub.publish(LEDmsg);
        LED2_pub.publish(LEDmsg);
        std::cout << "Robot er kørt ind i noget" << std::endl;
        geometry_msgs::Twist twistMsg;
        double angularspeed = 0.5;

        for (size_t i = 0; i < 13; i++)
        {
            twistMsg.linear.x = 0.0;
            twistMsg.angular.z = angularspeed;
            cmd_vel_pub.publish(twistMsg);
        }

        for (int i = 0; i < 5; i++)
        {
            twistMsg.linear.x = 0.1;
            twistMsg.angular.z = 0.0;
            cmd_vel_pub.publish(twistMsg);
            ros::Duration(0.5).sleep();
        }

        for (size_t i = 0; i < 13; i++)
        {
            twistMsg.linear.x = 0.0;
            twistMsg.angular.z = angularspeed;
            cmd_vel_pub.publish(twistMsg);
        }
        
        ros::Duration(0.5).sleep();
        LEDmsg.value = 1;
        LED1_pub.publish(LEDmsg);
        LED2_pub.publish(LEDmsg);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pubsub");
    srand(time(NULL));
    ros::NodeHandle n;
    ros::NodeHandle m;
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>
    ("/cmd_vel_mux/input/teleop", 1);
    LED1_pub = n.advertise<kobuki_msgs::Led>
    ("/mobile_base/commands/led1", 1);
    LED2_pub = n.advertise<kobuki_msgs::Led>
    ("/mobile_base/commands/led2", 1);
    bumperSub = n.subscribe
    ("/mobile_base/events/bumper", 1, bump);
    while (ros::ok())
    {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    return 0;
}
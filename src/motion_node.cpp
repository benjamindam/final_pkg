/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*@copyright Copyright 2017 Sudarshan Raghunathan
*@file motion_node.cpp
*@author Sudarshan Raghunathan
*@brief  Ros node to read direction to move in and publish velocity to turtlebot
*/
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include <nav_msgs/Odometry.h>
#include <iostream>
#include "linefollowertest/pos.h"
#include "linefollowertest/docking.h"

class turtlebot {
 public:
    int dir;  /// Direction message to read published directions
/**
*@brief Callback used to subscribe to the direction message published by the Line detection node
*@param msg is the custom message pos which publishes a direction int between 0 and 3
*@return none
*/
    void dir_sub(linefollowertest::pos msg);
/**
*@brief Function to publish velocity commands based on direction
*@param velocity is the twist 
*@param pub is used to publish the velocity commands to the turtlebot
*@param rate is the ros loop rate for publishing the commands
*@return none
*/
    void vel_cmd(geometry_msgs::Twist &velocity,
     ros::Publisher &pub, ros::Rate &rate);
};



double positionX{};
double positionY{};
void odomCallback(const nav_msgs::Odometry &msg)
{
    positionX = msg.pose.pose.position.x;
    positionY = msg.pose.pose.position.y;
    std::cout << positionX << "  " << positionY << std::endl;
}
void turtlebot::dir_sub(linefollowertest::pos msg) {
    turtlebot::dir = msg.direction; //Funktion som tager msg.direction, som bliver published af linedetect.cpp og ligger i 
                                    //turtlebot::dir
}

void turtlebot::vel_cmd(geometry_msgs::Twist &velocity,
 ros::Publisher &pub, ros::Rate &rate) {
    // If direction is left
    if (turtlebot::dir == 0) { //Hvis linedetect.cpp ser at linjen kører til venstre, bliver turtlebot::dir = 0
        velocity.linear.x = 0.1; //Fortæller at variabel velocity.linear.x skal være lig 0.1
        velocity.angular.z = 0.15; //Fortæller at variabel velocity.angular.z skal være lig 0.15
        pub.publish(velocity); //Publisher beskeden velocity til pub, som er publisheren til turtlebotten
        rate.sleep(); //Pauser programmet til næste cyklus
        ROS_INFO_STREAM("Turning Left"); //Printer i terminalen at robotten kører mod venstre
    }
    // If direction is straight
    if (turtlebot::dir == 1) {//Hvis linedetect.cpp ser at linjen kører ligeud, bliver turtlebot::dir = 1
        velocity.linear.x = 0.15;//Fortæller at variabel velocity.linear.x skal være lig 0.15
        velocity.angular.z = 0; //Fortæller at variabel velocity.angular.z skal være lig 0
        pub.publish(velocity);//Publisher beskeden velocity til pub, som er publisheren til turtlebotten
        rate.sleep();//Pauser programmet til næste cyklus
        ROS_INFO_STREAM("Straight");//Printer i terminalen at robotten kører ligeud
    }
    // If direction is right
    if (turtlebot::dir == 2) {//Hvis linedetect.cpp ser at linjen kører til højre, bliver turtlebot::dir = 2
        velocity.linear.x = 0.1;//Fortæller at variabel velocity.linear.x skal være lig 0.1
        velocity.angular.z = -0.15;//Fortæller at variabel velocity.angular.z skal være lig -0.15
        pub.publish(velocity);//Publisher beskeden velocity til pub, som er publisheren til turtlebotten
        rate.sleep();//Pauser programmet til næste cyklus
        ROS_INFO_STREAM("Turning Right");//Printer i terminalen at robotten kører mod højre
    }
    // If robot has to search
    if (turtlebot::dir == 3) {//Hvis linedetect.cpp ikke kan finde en linje, bliver turtlebot.dir = 3
        velocity.linear.x = 0;//Fortæller at variabel velocity.linear.x skal være lig 0
        velocity.angular.z = 0.25;//Fortæller at variabel velocity.angular.z skal være lig 0.25
        pub.publish(velocity);//Publisher beskeden velocity til pub, som er publisheren til turtlebotten
        rate.sleep();//Pauser programmet til næste cyklus
        ROS_INFO_STREAM("Searching");//Printer i terminalen at robotten kører mod højre
    }
 }
int main(int argc, char **argv)
{

    // Initializing node and object
    ros::init(argc, argv, "Velocity"); // Starter en ROS node med navn "Velocity"
    ros::NodeHandle n;
    turtlebot bot;                 //Laver et class objekt med navn bot. Denne class er oprettet i turtlebot.hpp
    geometry_msgs::Twist velocity; //Opretter en variabel med type geometry_msgs::Twist med navn velocity
    linefollowertest::docking dockMsg;
    ros::Subscriber sub = n.subscribe("/direction", 1, &turtlebot::dir_sub, &bot); //Opretter en subscriber på topic /direction
    ros::Subscriber odomSub = n.subscribe("/odom", 1, odomCallback);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10); //Opretter en publisher som publisher til /cmd_vel_mux/input/teleop
    ros::Publisher dockPub = n.advertise<linefollowertest::docking>("/docking/event", 10);
    ros::Rate rate(10);//sætter opdateringshastighed til 10hz
    bool goal = false; 
    while (ros::ok())
    {
        std::cout << goal << std::endl;
        if(goal == false)
        {
            
            /*
I include filen turtlebot.hpp er følgende funktion deklereret void vel_cmd(geometry_msgs::Twist &velocity, 
ros::Publisher &pub, ros::Rate &rate); 
Vi skal altså sende en besked med type geometry:msgs::Twist, en publisher og en ros::rate. 
Funktionen er i turtlebot.cpp
*/
            
            bot.vel_cmd(velocity, pub, rate);

            if (positionX < 6.8 && positionX > 6.2 )
            {
                if (positionY > 0 && positionY < 2)
                {
                    goal = true;
                    dockMsg.docking = 1;
                    dockPub.publish(dockMsg);
                }
            }
            rate.sleep(); //pauser programmet indtil næste cyklus
            ros::spinOnce(); //Kalder callback funktionerne, her turtlebot::dir_sub, som ligger i turtlebot.cpp
        }
    }
    return 0;
}

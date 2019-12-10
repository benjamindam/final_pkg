//HER KOMMER DEN ENDELIGE BATTERIKODE (UNDER BEHANDLING AF BENJAMIN)

#include <iostream>
#include <ros/ros.h>
#include <kobuki_msgs/SensorState.h>
#include <std_msgs/String.h>
#include <kobuki_msgs/PowerSystemEvent.h>
#include <geometry_msgs/Twist.h>
#include <math.h> 
#include <sstream>

int hiLo = 0;                                           //hiLo er tallet vi publisher til sidst

kobuki_msgs::SensorState msg;

int chargestateCallback(const kobuki_msgs::SensorState & chg)   //callback på events for turtlebottenbotten
{                  
    
    int chargingState=0;
    chargingState = chg.charger;                        //der findes mellem 0-6 events omkring ladning. 0=lader ikke. 2=fuldopladt. 6=lader.

    return chargingState;                               //retunerer hvad event den opfanger den er ved
}

void batteryCallback(const kobuki_msgs::SensorState & msg)
{
    float batt = msg.battery;                           //batt er batteriets spænding i dV
    
    if(chargestateCallback(msg) == 2)                   //2 betyder fuldopladt
    {
        hiLo = 0;                                       //0 betyder vent
        
        //herunder kører den bagud
        ros::NodeHandle n;
        ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1); //for sensors the value after , should be higher to get a more accurate result (queued)
        ros::Rate rate(10); 
        ros::Time start = ros::Time::now();
        while(ros::Time::now() - start < ros::Duration(1.0)) //kør NU, i 1 sekund
        {
            geometry_msgs::Twist move;
            move.linear.x = -0.1;                            //kør bagud (-) med hastigheden (0.1)
            movement_pub.publish(move);
        }
    }

    else if (chargestateCallback(msg) == 0)             //0 betyder den ikke lader
    {
        
        if(batt < 145)                      //når spændingen er under 135 dV
        {
            
            hiLo = 1;                       //1 betyder kør til dock

        }

        else if(batt >= 145)                //når spændingen er over eller lige med 135 dV
        {

            hiLo = 2;                       //2 betyder alt er okay
            
        }
        
    }

    else                                    //hvis den ikke er 0 eller 2/hvis den ikke er fuldopladt eller ikke er uden for laderen
    {
        hiLo = 0;
    }

}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery1");
 
    ros::NodeHandle n;
 
    ros::Subscriber sub = n.subscribe("/mobile_base/sensors/core", 10, batteryCallback);

    ros::Publisher battery_pub = n.advertise<std_msgs::String>("lav_batteri",1);

    kobuki_msgs::PowerSystemEvent msg;

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
    std_msgs::String msg;

    std::stringstream ss;
    ss << hiLo;                                   //vi publisher hvad hiLo er (0, 1 eller 2)
    msg.data = ss.str();
   
    std::cout << msg.data.c_str() << std::endl;

    battery_pub.publish(msg); 

    ros::spinOnce();

    loop_rate.sleep();
    }

    return 0;
}

  

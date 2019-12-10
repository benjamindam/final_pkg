#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>

int main(void) 
{

    const char *cmde = "gnome-terminal -x sh -c \"roslaunch kobuki_auto_docking activate.launch --screen\"";
    system(cmde);

    return 0;
}


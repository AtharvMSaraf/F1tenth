#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include "F1code/error.h"
#include <std_msgs/Float64.h>

class errorGen{
    private:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
    public:
    errorGen(){
        sub = n.subscribe("/scan",1000,&errorGen::calculateError,this);
        pub = n.advertise<F1code::error>("errorGenerated",1000);
    }
    void calculateError(sensor_msgs::LaserScan data);
    double sendError(float a,float b, int theta);  
};

void errorGen::calculateError(sensor_msgs::LaserScan data){
    float a = data.ranges[480];
    float b = data.ranges[270];
    int theta = 70;
    sendError(a,b, theta);
}

double errorGen::sendError(float a,float b, int theta){
    double target = 1.00;
    double alpha = atan((a*cos(theta) - b)/(a*sin(theta)));
    double AB = b*cos(alpha);
    double CD = AB + 0.2*sin(alpha);
    double error = target-CD; //set the distance from the wall from here
    // std::cout<<error<<std::endl;
    F1code::error er;
    er.error.data = error;
    er.AB.data = AB;
    er.CD.data = CD;
    pub.publish(er);
    return error;
}

int main(int argc, char **argv){
    ros::init(argc,argv,"errorGen");
    errorGen r;
    ros::spin();
    return 0;
}
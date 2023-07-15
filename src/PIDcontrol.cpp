#include <ros/ros.h>
#include <F1code/error.h>
#include "ackermann_msgs/AckermannDriveStamped.h"

float prv_error= 0;

class pidControl
{
    private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;

    public:
    pidControl(){
        pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive",1000);
        sub = n.subscribe("/errorGenerated",1000, &pidControl::pidTune,this);
    }
    void pidTune(F1code::error er);
};

void pidControl::pidTune(F1code::error er){
    ackermann_msgs::AckermannDriveStamped drive;
    drive.drive.speed = 1; 
    float stearAngle,
    kp =10,
    kd = 2,
    ki = 0.002;
    static float error_sum;
    stearAngle = (kp*er.error.data) + kd*(er.error.data-prv_error) ;
    if(+ ki*(error_sum += er.error.data)< 1){
        std::cout<<"i did include integral"<<std::endl;
        stearAngle += + ki*(error_sum += er.error.data);
    }
    std::cout<<"error diff "<<er.error.data-prv_error<<std::endl; 
    std::cout<<"stearing angle "<<stearAngle<<std::endl;    
    prv_error = er.error.data;

    if(stearAngle>-100 && stearAngle<100){
        drive.drive.steering_angle = stearAngle;
        
    }
    pub.publish(drive);
    
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pidControl");
    pidControl r;
    ros::spin();
    return 0;
}



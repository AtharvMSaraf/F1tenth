#include <ros/ros.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <stdio.h>
#include <cmath>

// using namespace std;

template <typename S>
std::ostream& operator<<(std::ostream& os,
                    const std::vector<S>& vector)
{
    // Printing all the elements
    // using <<
    for (auto element : vector) {
        os << element << " ";
    }
    return os;
}

class f1tenth
{
    private:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;

    public:
        f1tenth(){
            //defining publisher
            pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000);

            //defining subscribers
            sub = n.subscribe("/scan",1000,&f1tenth::race,this);   
        }

        void race(sensor_msgs::LaserScan scan){
            ackermann_msgs::AckermannDriveStamped drive;
            
            // std::cout<<scan.ranges[810]<<std::endl;
            
            for(int i=525;i<556;i++){
                // if(scan.ranges[i]<0.4){
                //     std::cout<<"The distance is"<<scan.ranges[525]<<"i mam breaking, speed = "<<drive.drive.speed<<std::endl;
                //     drive.drive.speed-=0.01;
                //     pub.publish(drive);
                // }
                if(scan.ranges[i]<0.5){
                    std::cout<<"emergency break applied"<<std::endl;
                    drive.drive.speed = 0;
                    drive.drive.acceleration = 0;
                    pub.publish(drive);
                }
                else{
                    wallFollow(scan,drive);

                }
            }
        }
        

        void wallFollow(sensor_msgs::LaserScan scan, ackermann_msgs::AckermannDriveStamped drive){
            
            float minLeft = 3;
            for(int i=540;i<=860;i++){
                if(scan.ranges[i]<minLeft){
                    minLeft=scan.ranges[i];
                }
            }
            std::cout<<minLeft<<std::endl;
            if(minLeft<1){
                std::cout<<"entered the wall follow turning right"<<std::endl;
                drive.drive.speed = 1;
                drive.drive.steering_angle = -minLeft;
            }
            else if(minLeft>1.1){
                std::cout<<"entered the wall follow turning left"<<std::endl;
                drive.drive.speed = 1;
                drive.drive.steering_angle = minLeft;
            }
            else{
                drive.drive.speed=2;
                drive.drive.steering_angle=0;
            }
            pub.publish(drive);


            }
        
};

int main(int argc, char **argv){
    ros::init(argc,argv,"lidarLap");
    f1tenth r; 
    ros::spin();
    return 0;
}
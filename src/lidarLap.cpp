#include <ros/ros.h>
#include "ackermann_msgs/AckermannDriveStamped.h"   
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "nav_msgs/Odometry.h"

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
        ros::Subscriber sub2;

    public:
        f1tenth(){
            //defining publisher
            pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000);

            //defining subscribers
            sub = n.subscribe("/scan",1000,&f1tenth::race,this); 

            // defining second subsriber
            sub2 = n.subscribe("/odom",1000,&f1tenth::odomcallback,this);
 
        }

        void race(sensor_msgs::LaserScan scan);
        void wallFollow(sensor_msgs::LaserScan scan, ackermann_msgs::AckermannDriveStamped drive);
        void odomcallback(nav_msgs::Odometry);
        float velocity; 

};

void f1tenth::race(sensor_msgs::LaserScan scan){
    ackermann_msgs::AckermannDriveStamped drive;
            
            // std::cout<<"range= "<<scan.ranges[810]<<" ttc= "<<ttc<<std::endl;

            // float ttc_min = INFINITY;
            for(int i=525;i<556;i++){
                float ttc= scan.ranges[i]/-(velocity*cos(2*3.14159265359*((i*0.333)/360)));
                
                if(scan.ranges[i]<0.5){
                    std::cout<<"emergency break applied"<<std::endl;
                    drive.drive.speed = 0;
                    drive.drive.acceleration = 0;
                    pub.publish(drive);
                }
                else{
                    if (ttc<8){
                    drive.drive.speed= drive.drive.speed/2;
                    std::cout<<"deaccelrating velocity = "<<velocity<<std::endl;
                    pub.publish(drive);
                }
                    // wallFollow(scan,drive);
                    drive.drive.speed = 0.5;
                    pub.publish(drive);

                }
            }
}

void f1tenth::wallFollow(sensor_msgs::LaserScan scan, ackermann_msgs::AckermannDriveStamped drive){
            
            float minLeft = 3;
            for(int i=540;i<=950;i++){
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

void f1tenth::odomcallback(nav_msgs::Odometry odom){
    std::cout<<"i am in odometry"<<std::endl;
    velocity = odom.twist.twist.linear.x;
    std::cout<<"velocity  = "<<velocity<<std::endl;
    
}

int main(int argc, char **argv){
    ros::init(argc,argv,"lidarLap");
    f1tenth r; 
    ros::spin();
    return 0;
}
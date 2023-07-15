#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

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

class gapFollow{
    private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber sub2;

    public:
     gapFollow(){
        pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive",1000);
        sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1000,&gapFollow::callback,this);
        sub2 = n.subscribe<nav_msgs::Odometry>("/odom",1000,&gapFollow::odomCallback,this);
    }
    float current_angle;
    void callback(sensor_msgs::LaserScan data);
    void odomCallback(nav_msgs::Odometry odom);
    sensor_msgs::LaserScan disparityCorection(sensor_msgs::LaserScan data);
    std::vector<float> findGap(sensor_msgs::LaserScan);
    float targetAngle(sensor_msgs::LaserScan data);
    void drive(float target_angle);
    
};

void gapFollow::callback(sensor_msgs::LaserScan data){
    double attack_angle = targetAngle(data);
    drive(attack_angle);

}

float gapFollow::targetAngle(sensor_msgs::LaserScan data){
    float range = 5.0;
    std::vector<int> gaps;
    int gap_start,gap_end,gap_length,no_of_gaps=1, max_gap=0,gap_midpoit;
    for(int i = 270;i<809;){
        data.ranges[i] = data.ranges[i]>range?range:data.ranges[i];
        // std::cout<<data.ranges[i]<<std::endl;
        if(std::abs(( range - data.ranges[i+1])<0.2)){
            gap_start = i;
            while(data.ranges[gap_start]-data.ranges[i]<0.2){
                i++;
            }
            gap_end = i;
            gap_length =  gap_end - gap_start;
            if(gap_length>max_gap){
                // std::cout<<"the max len now is = "<<max_gap<<std::endl;
                max_gap = gap_length;
                gap_midpoit = (gap_end+gap_start)/2;
                
                // std::cout<<"gap_midpoint = "<<gap_midpoit<<std::endl;

            }
            if(gap_length > 8){
                std::cout<<" gap no. "<<no_of_gaps<<" found starts from:to  = "<<gap_start<<" : "<<gap_end<<"gap length = "<<gap_length<<std::endl;
                gaps.push_back(gap_start);
                gaps.push_back(gap_end);
                gaps.push_back(gap_length);
                no_of_gaps++;
            }  
        }
        i++;
    }
    float target_angle = ((float)(360.0/1080.0))*gap_midpoit;
    // std::cout<<"the max len now is = "<<max_gap<<std::endl;
    std::cout<<"gap_midpoint = "<<gap_midpoit<<std::endl;
    // std::cout<<"target angle = "<<target_angle - 180 <<std::endl;
    return target_angle-180;
}

void gapFollow::drive(float target_angle){
    ackermann_msgs::AckermannDriveStamped d;
    d.drive.speed = 1.0;
    double angle_diff = std::abs(target_angle-current_angle);
    if(angle_diff>2.0){
        if((target_angle>current_angle)>0){
            // std::cout<<"turning clockwise"<<std::endl;
            d.drive.steering_angle = 0.5;
        }
        else{
            // std::cout<<"turining anticlockwise"<<std::endl;
            d.drive.steering_angle = -0.5;
        }
    }
    else{
        d.drive.steering_angle =0;
    }
    pub.publish(d);

}

void gapFollow::odomCallback(nav_msgs::Odometry odom){
    tf::Quaternion q(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    // std::cout<<"yaw = "<<yaw * (180.0/3.141592653)<<std::endl;
    current_angle = yaw * (180.0/3.141592653);
}

int main(int argc, char **argv){
    ros::init(argc, argv,"gappFollow");
    gapFollow r;
    ros::spin();
    // while(ros::ok){
    //     ros::init(argc, argv,"gappFollow");
    //     gapFollow r;
    //     ros::spinOnce();
    // }
    return 0;
}
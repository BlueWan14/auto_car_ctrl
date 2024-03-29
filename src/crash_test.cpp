#include <ros/ros.h>
#include <auto_car_ctrl/motors.h>
#include <auto_car_ctrl/rosBool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


// DÉCLARATION DES VARIABLES a=====================================================================================
#define car_size 0.09

const float pi = std::acos(-1),                                                 // Calcul de pi
            left_edge_vision = pi / 2,
            right_edge_vision = - pi / 2,
            left_crash_vision = pi / 6,
            right_crash_vision = - pi / 6;
float dist_follow_wall = 0.1,
      speed_max = 100.0;
auto_car_ctrl::rosBool crash;

// Other sub and publisher ---------------------------------------------------------------------------------------
ros::Subscriber lidar_sub;
ros::Subscriber goodWay_sub;
ros::Publisher crash_pub;


// PROTOTYPES ====================================================================================================
void goodWayCallBack(const auto_car_ctrl::rosBool &);
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &);


// MAIN ==========================================================================================================
int main(int argc, char** argv) {
    ROS_INFO("Starting car control...");

    crash.answer = false;

    ROS_INFO("Init ROS...");
    ros::init(argc, argv, "auto_car_ctrl");                                     // Initalisation de ROS
    ROS_INFO("Complete.");

    ROS_INFO("Subscribers and publishers creation...");
    ros::NodeHandle nh;                                                         // Communication ROS
    ros::param::get("/dist_follow_wall", dist_follow_wall);
    ros::param::get("/speed_max", speed_max);

    // Création des subscribers ----------------------------------------------------------------------------------
    lidar_sub = nh.subscribe("/scan", 100, &lidarCallback);
    goodWay_sub = nh.subscribe("/auto_car/crash/goodway", 100, &goodWayCallBack);
    // Création des publishers -----------------------------------------------------------------------------------
    crash_pub = nh.advertise<auto_car_ctrl::rosBool>("auto_car/crash/iscrashed", 100);
    ROS_INFO("Complete.");

    ROS_INFO("Starting loop.");
    ros::spin();                                                                // Boucle de fonctionnement du package
}


// DÉFINITIONS DE FONCTIONS ====================================================================================== 
void goodWayCallBack(const auto_car_ctrl::rosBool &goodWay_msg) {
    if(goodWay_msg.answer)
        crash.answer = false;
}


/*
description : Fonction callback appelée à chaque modification du topic /scan
paramètre : (const, sensor_msgs::LaserScan::ConstPtr, pointeur) scan_msg : message reçu.
*/
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    float min_range = 12.0, angle = 0.0;
    geometry_msgs::Twist cmd;

    crash.header.frame_id = "crash";
    crash.header.stamp = ros::Time::now();
    
    for(int i = 0; i <= scan_msg->ranges.size(); i++) {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;            // On calcul l'angle associé
        
        if((left_edge_vision > angle) && (angle > right_edge_vision) && (scan_msg->ranges[i] < min_range) && (scan_msg->ranges[i] > car_size)) {
            min_range = scan_msg->ranges[i];                                    // Sauvegarde de la plus grande distance
        }
    }
    if((min_range < dist_follow_wall) && (left_crash_vision > angle) && (angle > right_crash_vision)) {
        cmd.linear.x = - speed_max;
        crash.answer = true;
    } else {
        cmd.linear.x = 0.0;
        crash.answer = false;
    }

    crash_pub.publish(crash);
}

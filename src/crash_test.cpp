#include <ros/ros.h>
#include <auto_car_ctrl/motors.h>
#include <auto_car_ctrl/rosBool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


// DÉCLARATION DES VARIABLES a=====================================================================================
#define car_size 0.09

const float pi = std::acos(-1),                                                 // Calcul de pi
            left_crash_vision = pi / 6,
            right_crash_vision = - pi / 6;
float dist_follow_wall = 0.1,
      speed_max = 100.0;
auto_car_ctrl::rosBool crash;
bool rviz, isGoodWay;

// Other sub and publisher ---------------------------------------------------------------------------------------
visualization_msgs::Marker crash_marker;
ros::Subscriber motor_sub;
ros::Subscriber lidar_sub;
ros::Subscriber goodWay_sub;
ros::Publisher crash_pub;
ros::Publisher crash_marker_pub;


// PROTOTYPES ====================================================================================================
void motorCallBack(const auto_car_ctrl::motors &);
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
    ros::param::get("/rviz", rviz);

    // Création des subscribers ----------------------------------------------------------------------------------
    motor_sub = nh.subscribe("/auto_car/arduino/mot", 100, &motorCallBack);
    lidar_sub = nh.subscribe("/scan", 100, &lidarCallback);
    goodWay_sub = nh.subscribe("/auto_car/crash/goodway", 100, &goodWayCallBack);
    // Création des publishers -----------------------------------------------------------------------------------
    crash_pub = nh.advertise<auto_car_ctrl::rosBool>("auto_car/crash/iscrashed", 100);
    if(rviz) {
        ROS_INFO("Markers configuration...");
        // Marker "position crash" -----------------------------------------------------------------------------------
        crash_marker.ns = "crash";
        crash_marker.id = 0;
        crash_marker.type = visualization_msgs::Marker::POINTS;
        crash_marker.action = visualization_msgs::Marker::ADD;
        crash_marker.pose.orientation.w = 0.0;
        crash_marker.scale.x = 0.05;
        crash_marker.scale.y = 0.05;
        crash_marker.scale.z = 0.05;
        crash_marker.color.a = 1.0;
        crash_marker.color.r = 1.0;
        crash_marker.color.b = 0.875;
        crash_marker_pub = nh.advertise<visualization_msgs::Marker>("auto_car/markers/crash", 100);
    }
    ROS_INFO("Complete.");

    ROS_INFO("Starting loop.");
    ros::spin();
}


// DÉFINITIONS DE FONCTIONS ====================================================================================== 
void goodWayCallBack(const auto_car_ctrl::rosBool &goodWay_msg) {
    isGoodWay = goodWay_msg.answer;
}

// DÉFINITIONS DE FONCTIONS ======================================================================================
/*
description : Fonction callback appelée à chaque modification du topic /auto_car/lidar_process
paramètre : (const, auto_car_ctrl::rosFloat::ConstPtr, pointeur) angle_msg : message reçu.
*/
void motorCallBack(const auto_car_ctrl::motors &motor_msg) {
    if(crash.answer && ((motor_msg.rearObstacle < 10.0) || isGoodWay))
        crash.answer = false;
}

/*
description : Fonction callback appelée à chaque modification du topic /scan
paramètre : (const, sensor_msgs::LaserScan::ConstPtr, pointeur) scan_msg : message reçu.
*/
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    float angle = 0.0;
    geometry_msgs::Point p;

    crash_marker.points.clear();

    crash.header.frame_id = "crash";
    crash.header.stamp = ros::Time::now();
    
    for(int i = 0; i <= scan_msg->ranges.size(); i++) {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;            // On calcul l'angle associé
        
        if(((right_crash_vision < angle) && (angle < left_crash_vision)) && (car_size < scan_msg->ranges[i]) && (scan_msg->ranges[i] < dist_follow_wall)) {
            crash.answer = true;
            if(rviz) {
                p.x = scan_msg->ranges[i] * std::cos(angle);
                p.y = scan_msg->ranges[i] * std::sin(angle);
                crash_marker.points.push_back(p);
            } else {
                break;
            }
        }
    }

    if(rviz) {
        crash_marker.header.frame_id = scan_msg->header.frame_id;
        crash_marker.header.stamp = ros::Time::now();
        crash_marker_pub.publish(crash_marker);
    }

    crash_pub.publish(crash);
}

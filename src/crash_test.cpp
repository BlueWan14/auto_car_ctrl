#include <ros/ros.h>
#include <auto_car_ctrl/motors.h>
#include <auto_car_ctrl/rosBool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


// DÉCLARATION DES VARIABLES =====================================================================================
#define car_size 0.09

const float pi = std::acos(-1),                                                 // Calcul de pi
            left_edge_vision = pi / 2,
            right_edge_vision = - pi / 2;
float dist_follow_wall = 0.1,
      speed_max = 100.0;
bool start_flag = false;
auto_car_ctrl::rosBool crash;

// Other sub and publisher ---------------------------------------------------------------------------------------
ros::Subscriber motor_sub;
ros::Subscriber lidar_sub;
ros::Publisher crash_pub;
ros::Publisher cmd_pub;


// PROTOTYPES ====================================================================================================
void motorCallBack(const auto_car_ctrl::motors &);
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
    motor_sub = nh.subscribe("/auto_car/arduino/mot", 100, &motorCallBack);
    lidar_sub = nh.subscribe("/scan", 100, &lidarCallback);
    // Création des publishers -----------------------------------------------------------------------------------
    crash_pub = nh.advertise<auto_car_ctrl::rosBool>("auto_car/crash", 100);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("auto_car/cmd_vel", 100);
    ROS_INFO("Complete.");

    ROS_INFO("Starting loop.");
    ros::spin();                                                                // Boucle de fonctionnement du package
}


// DÉFINITIONS DE FONCTIONS ======================================================================================
/*
description : Fonction callback appelée à chaque modification du topic /auto_car/lidar_process
paramètre : (const, auto_car_ctrl::rosFloat::ConstPtr, pointeur) angle_msg : message reçu.
*/
void motorCallBack(const auto_car_ctrl::motors &motor_msg) {
    crash.header.frame_id = "crash";
    crash.header.stamp = ros::Time::now();

    if((motor_msg.vel.linear.x != 0) && (motor_msg.coder == 0)) {
        crash.answer = true;
    }

    crash_pub.publish(crash);
}

/*
description : Fonction callback appelée à chaque modification du topic /scan
paramètre : (const, sensor_msgs::LaserScan::ConstPtr, pointeur) scan_msg : message reçu.
*/
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    float min_range = 12.0, angle = 0.0;
    geometry_msgs::Twist cmd;

    cmd.angular.z = 0.0;
    
    if(crash.answer && start_flag) {
        for(int i = 0; i <= scan_msg->ranges.size(); i++) {                         // Pour chaque distance mesurée
            angle = scan_msg->angle_min + i * scan_msg->angle_increment;            // On calcul l'angle associé

            // Recherche de la distance la plus petite dans le champ de vision ---------------------------------------
            if((left_edge_vision > angle) && (angle > right_edge_vision) && (scan_msg->ranges[i] < min_range) && (scan_msg->ranges[i] > car_size)) {
                min_range = scan_msg->ranges[i];                                    // Sauvegarde de la plus grande distance
            }
        }

        if(min_range < dist_follow_wall) {
            cmd.linear.x = -speed_max;
        } else {
            cmd.linear.x = 0.0;
            crash.answer = false;
        }
    } else {
        start_flag = true;
    }
}

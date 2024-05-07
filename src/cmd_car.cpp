#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <auto_car_ctrl/motors.h>
#include <unistd.h>

// DÉCLARATION DES VARIABLES =====================================================================================
const float left_angle_max = 540,   // Extrémité gauche du champ de vision
            right_angle_max = 179,  // Extrémité droite du champ de vision
            middle_angle_left = 360,
            middle_angle_right = 359,
            dist_follow_wall = 0.1;
const float Vrotmax = 9428.57,
            d = 0.065,
            pi = std::acos(-1);

float speed = 0.0;                  // Vitesse renvoyée par la carte Arduino
bool rouler = false;

ros::Subscriber lidar_sub;          // Nouveau subscriber
ros::Subscriber vel_sub;            // Nouveau subscriber
ros::Publisher cmd_vel;             // Nouveau publisher


// PROTOTYPES ====================================================================================================
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
void motCallback(const auto_car_ctrl::motors &vel_msg);


// MAIN ==========================================================================================================
int main(int argc, char** argv) {
    ROS_INFO("Starting car control...");

    ROS_INFO("Init ROS...");
    ros::init(argc, argv, "lidar_listener");    // Initalisation de ROS
    ROS_INFO("Complete.");

    ROS_INFO("RViz setup...");
    //initRViz();                                 // Initialisation de RViz
    ROS_INFO("Complete.");

    ROS_INFO("Subscribers and publishers creation...");
    ros::NodeHandle nh;                         // Communication ROS
    // Création d'un subscriber sur le topic /scan avec un buffer de 100
    lidar_sub = nh.subscribe("/scan", 100, &lidarCallback);
    vel_sub = nh.subscribe("/auto_car/mot/vel", 100, &motCallback);
    // Création d'un publisher selon la forme geometry_msgs::Twist sur le topic auto_car/mot/cmd_vel avec un buffer de 100
    cmd_vel = nh.advertise<geometry_msgs::Twist>("auto_car/cmd_vel", 100);
    ROS_INFO("Complete.");

    ROS_INFO("Starting loop.");
    ros::spin();                                // Boucle de fonctionnement du package
}

// DÉFINITIONS DE FONCTIONS ======================================================================================
/*
description : Fonction callback appelée à chaque modification du topic /scan
paramètre : (const, sensor_msgs::LaserScan::ConstPtr, pointeur) scan_msg : message reçu.
*/
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    geometry_msgs::Twist cmd;
    float range_max = 0.0,
        lidar_pos = 0.0, cmd_rot = 0.0;
    int i_range_max = 0,
        follow_left = 0, follow_right = 0;
    
    for(int i = 0; i <= scan_msg->ranges.size(); ++i) {
        if((i > right_angle_max) && (i < left_angle_max) && (scan_msg->ranges[i] > range_max)) {
            range_max = scan_msg->ranges[i];                                                    // Distance max mesurée
            i_range_max = i;                                                                    // Nombre d'incrément pour obtenir l'angle
        }
        if((i == right_angle_max) && (scan_msg->ranges[i] < dist_follow_wall)) {
            follow_right = i;
        } else if((i == left_angle_max) && (scan_msg->ranges[i] < dist_follow_wall)) {
            follow_left = i;
        }
    }

    if (speed = 0) {
        if (rouler) {
            cmd.angular.z = (-cmd_rot) * 180 / pi; 		//tourner à l'opposé 
            cmd.linear.x = -10; 					//reculer
            cmd_vel.publish(cmd);
            sleep(2);						//reculer pendant 2s
            cmd.angular.z = 0.0;
            cmd.linear.x = 10;
            cmd_vel.publish(cmd);
        } else {
            rouler = true;
        }
    } else {
        if(follow_right != 0) {
            lidar_pos = scan_msg->ranges[follow_right] * std::sin(scan_msg->angle_min + follow_right * scan_msg->angle_increment) - dist_follow_wall;
            cmd_rot = std::atan(lidar_pos/speed);                               // Rotation des roues (en rad)
            cmd.angular.z = cmd_rot * 180 / pi;                                 // Conversion en °
        } else if (follow_left != 0) {
            lidar_pos = scan_msg->ranges[follow_left] * std::sin(scan_msg->angle_min + follow_left * scan_msg->angle_increment) + dist_follow_wall;
            cmd_rot = std::atan(lidar_pos/speed);                               // Rotation des roues (en rad)
            cmd.angular.z = cmd_rot * 180 / pi;                                 // Conversion en °
        } else {
            cmd.angular.z = (scan_msg->angle_min + i_range_max * scan_msg->angle_increment) * 180 / pi; // Rotation des roues (en °)
        }
        cmd.linear.x = 20;                                                                          // Vitesse de déplacement de la voiture (en %)
    }
    cmd_vel.publish(cmd);                                                                       // Publication sur le topic auto_car/cmd_vel
}

/*
description : Fonction callback appelé à chaque modification du topic /auto_car_ctrl/mot/vel
paramètre : (const, auto_car_ctrl::motors, pointeur) vel_msg : message reçu.
*/
void motCallback(const auto_car_ctrl::motors &vel_msg) {
    speed = vel_msg.vel.linear.x * d * pi / 156;                            // Calcul de la vitesse de la voiture (m/s)
}



#include <ros/ros.h>
#include <auto_car_ctrl/rosBool.h>
#include <auto_car_ctrl/rosFloat.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>


// DÉCLARATION DES VARIABLES =====================================================================================
const float pi = std::acos(-1),                                                 // Calcul de pi
            left_edge_vision = pi / 2,
            right_edge_vision = - pi / 2;
float max_range, too_close,
      speed_max = 100.0,
      way = 1.0;
int angle_max_left = 40,
    angle_max_right = -40;
bool start;

// Other sub and publisher ---------------------------------------------------------------------------------------
ros::Subscriber processed_sub;
ros::Subscriber way_sub;
ros::Subscriber start_sub;
ros::Publisher cmd_pub;

// PROTOTYPES ====================================================================================================
void lidarCallBack(const auto_car_ctrl::rosFloat &);
void wayCallBack(const auto_car_ctrl::rosFloat &);
void startCallBack(const auto_car_ctrl::rosBool &);

// MAIN ==========================================================================================================
int main(int argc, char** argv) {
    ROS_INFO("Starting car control...");

    ROS_INFO("Init ROS...");
    ros::init(argc, argv, "auto_car_ctrl");                                     // Initalisation de ROS
    ROS_INFO("Complete.");

    ROS_INFO("Subscribers and publishers creation...");
    ros::NodeHandle nh;                                                         // Communication ROS
    ros::param::get("/angle_max_left", angle_max_left);
    ros::param::get("/angle_max_right", angle_max_right);
    ros::param::get("/speed_max", speed_max);

    // Création des subscribers ----------------------------------------------------------------------------------
    processed_sub = nh.subscribe("/auto_car/cmd/lidar_process", 100, &lidarCallBack);
    way_sub = nh.subscribe("/auto_car/cmd/way", 100, &wayCallBack);
    start_sub = nh.subscribe("/start", 100, &startCallBack);

    // Création des publishers -----------------------------------------------------------------------------------
    cmd_pub = nh.advertise<geometry_msgs::Twist>("auto_car/arduino/cmd_vel", 100);
    ROS_INFO("Complete.");

    ROS_INFO("Starting loop.");
    ros::spin();                                                                // Boucle de fonctionnement du package
}


// DÉFINITIONS DE FONCTIONS ======================================================================================
/*
description : Fonction callback appelée à chaque modification du topic /auto_car/lidar_process
paramètre : (const, auto_car_ctrl::rosFloat::ConstPtr, pointeur) angle_msg : message reçu.
*/
void lidarCallBack(const auto_car_ctrl::rosFloat &angle_msg) {
    geometry_msgs::Twist cmd;

    if(start) {
        // Commande de rotation des roues (en °)
        cmd.angular.z = angle_msg.val * 180 / pi;                                   // Suivi de l'objectif

        // Commande de vitesse du moteur (en %)
        if(cmd.angular.z < angle_max_right) {
            // On ralenti pour tourner plus vite à droite
            cmd.linear.x = speed_max - ((cmd.angular.z - angle_max_right) * speed_max / (-102.5 - angle_max_right));
        } else if(cmd.angular.z > angle_max_left) {
            // On ralenti pour tourner plus vite à gauche
            cmd.linear.x = speed_max - ((cmd.angular.z - angle_max_left) * speed_max / (102.5 - angle_max_left));
        } else {
            cmd.linear.x = way * speed_max;                                         // On met les gaz
        }
    } else {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
    }
    
    cmd_pub.publish(cmd);
}

void wayCallBack(const auto_car_ctrl::rosFloat &way_msg) {
    way = way_msg.val;
}


void startCallBack(const auto_car_ctrl::rosBool &start_msg) {
    start = start_msg.answer;
}

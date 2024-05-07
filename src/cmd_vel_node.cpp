#include <ros/ros.h>
#include <auto_car_ctrl/rosBool.h>
#include <auto_car_ctrl/rosFloat.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>


// DÉCLARATION DES VARIABLES =====================================================================================
const float pi = std::acos(-1),                                                 // Calcul de pi
            left_edge_vision = pi / 2,
            right_edge_vision = - pi / 2;
float max_range, too_close;
int angle_max_left = 25,
    angle_max_right = -20;
bool obstacle = true;

// Other sub and publisher ---------------------------------------------------------------------------------------
ros::Subscriber processed_sub;
ros::Publisher cmd_pub;


// PROTOTYPES ====================================================================================================
void lidarCallBack(const auto_car_ctrl::rosFloat &);


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

    // Création des subscribers ----------------------------------------------------------------------------------
    processed_sub = nh.subscribe("/auto_car/lidar_process", 100, &lidarCallBack);
    // Création des publishers -----------------------------------------------------------------------------------
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
void lidarCallBack(const auto_car_ctrl::rosFloat &angle_msg) {
    geometry_msgs::Twist cmd;

    // Commande de rotation des roues (en °)
    cmd.angular.z = angle_msg.val * 180 / pi;                           // Suivi de l'objectif

    // Commande de vitesse du moteur (en %)
    if(cmd.angular.z < angle_max_right) {
        // On ralenti pour tourner plus vite à droite
        cmd.linear.x = 100.0 - ((cmd.angular.z - angle_max_right) * 100.0 / (-91.5 - angle_max_right));
    } else if(cmd.angular.z > angle_max_left) {
        // On ralenti pour tourner plus vite à gauche
        cmd.linear.x = 100.0 - ((cmd.angular.z - angle_max_left) * 100.0 / (91.5 - angle_max_left));
    } else {
        cmd.linear.x = 100.0;                                           // On met les gaz
    }

    cmd_pub.publish(cmd);
}

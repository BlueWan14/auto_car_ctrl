#include <ros/ros.h>
#include <unistd.h>

// ROS messages libraries ----------------------------------------------------------------------------------------
#include <auto_car_ctrl/rosBool.h>
#include <auto_car_ctrl/rosFloat.h>
#include <auto_car_ctrl/CmdVel.h>


// DÉCLARATION DES VARIABLES =====================================================================================
float speed_max = 60.0,
      way = 1.0;
int angle_max_left = 40,
    angle_max_right = -40;

// ROS subscriber and publisher ----------------------------------------------------------------------------------
ros::Subscriber processed_sub;
ros::Subscriber way_sub;
ros::Publisher cmd_pub;

// PROTOTYPES ====================================================================================================
void lidarCallBack(const auto_car_ctrl::rosFloat &);
void wayCallBack(const auto_car_ctrl::rosFloat &);

// MAIN ==========================================================================================================
int main(int argc, char** argv) {
    ROS_INFO("Starting car control...");

    ROS_INFO("Init ROS...");
    ros::init(argc, argv, "auto_car_ctrl");                                     // Initalisation de ROS
    ROS_INFO("Complete.");

    ROS_INFO("Subscribers and publishers creation...");
    ros::NodeHandle nh;                                                         // Communication ROS

    // Réupération des arguments du roslaunch --------------------------------------------------------------------
    ros::param::get("/speed_max", speed_max);

    // Création des subscribers ----------------------------------------------------------------------------------
    processed_sub = nh.subscribe("/auto_car/cmd/lidar_process", 100, &lidarCallBack);
    way_sub = nh.subscribe("/auto_car/cmd/way", 100, &wayCallBack);

    // Création des publishers -----------------------------------------------------------------------------------
    cmd_pub = nh.advertise<auto_car_ctrl::CmdVel>("auto_car/arduino/cmd_vel", 100);
    ROS_INFO("Complete.");

    ROS_INFO("Starting loop.");
    ros::spin();                                                                // Lancement de la boucle ROS du noeud
}


// DÉFINITIONS DE FONCTIONS ======================================================================================
/*
description : Fonction callback appelée à chaque modification du topic "/auto_car/cmd/way"
paramètre : (const, auto_car_ctrl::rosFloat::ConstPtr, pointeur) way_msg : le message ROS reçu.
*/
void wayCallBack(const auto_car_ctrl::rosFloat &way_msg) {
    way = way_msg.val;
}

/*
description : Fonction callback appelée à chaque modification du topic "/auto_car/lidar_process"
paramètre : (const, auto_car_ctrl::rosFloat::ConstPtr, pointeur) angle_msg : le message ROS reçu.
*/
void lidarCallBack(const auto_car_ctrl::rosFloat &angle_msg) {
    auto_car_ctrl::CmdVel cmd;

    // Commande de rotation des roues (en °) ---------------------------------------------------------------------
    cmd.angular = angle_msg.val * 180 / pi;

    // Commande de vitesse du moteur (en %) ----------------------------------------------------------------------
    if(cmd.angular < angle_max_right) {
        // Si on tourne à droite, on ralenti proportionnellement la vitesse de la voiture
        cmd.linear = speed_max - ((cmd.angular - angle_max_right) * speed_max / (-102.5 - angle_max_right));
    } else if(cmd.angular > angle_max_left) {
        // De même si on va à gauche
        cmd.linear = speed_max - ((cmd.angular - angle_max_left) * speed_max / (102.5 - angle_max_left));
    } else {
        cmd.linear = speed_max;                                                 // Sinon  on met les gazs
    }
    cmd.linear = way * cmd.linear;                                              // On applique la direction (marche avant/arrière)
    
    cmd_pub.publish(cmd);                                                       // Publlication sur le topic "auto_car/arduino/cmd_vel"
}

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <auto_car_ctrl/motors.h>


// DÉCLARATION DES VARIABLES =====================================================================================
const float left_angle_max = 540;   // Extrémité gauche du champ de vision
const float right_angle_max = 179;  // Extrémité droite du champ de vision
const float middle_threshold = 0.1; // Sensibilité afin de considérer être au milieu

const float Vrotmax = 9428.57,
            d = 0.065,
            pi = std::acos(-1);

visualization_msgs::Marker marker;  // Points sur RViz
ros::Subscriber lidar_sub;          // Nouveau subscriber
ros::Subscriber vel_sub;            // Nouveau subscriber
ros::Publisher marker_pub;          // Nouveau publisher
ros::Publisher cmd_vel;             // Nouveau publisher

float speed = 0.0;                  // Vitesse renvoyée par la carte Arduino


// PROTOTYPES ====================================================================================================
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
void motCallback(const auto_car_ctrl::motors &vel_msg);
void initRViz();


// MAIN ==========================================================================================================
int main(int argc, char** argv) {
    ROS_INFO("Starting car control...");

    ROS_INFO("Init ROS...");
    ros::init(argc, argv, "lidar_listener");    // Initalisation de ROS
    ROS_INFO("Complete.");

    ROS_INFO("RViz setup...");
    initRViz();                                 // Initialisation de RViz
    ROS_INFO("Complete.");

    ROS_INFO("Subscribers and publishers creation...");
    ros::NodeHandle nh;                         // Communication ROS
    // Création d'un subscriber sur le topic /scan avec un buffer de 100
    lidar_sub = nh.subscribe("/scan", 100, &lidarCallback);
    vel_sub = nh.subscribe("/auto_car/mot/vel", 100, &motCallback);
    // Création d'un publisher selon la forme Marker sur le topic auto_car/lidar/marker avec un buffer de 100
    marker_pub = nh.advertise<visualization_msgs::Marker>("auto_car/lidar/markers", 100);
    // Création d'un publisher selon la forme geometry_msgs::Twist sur le topic auto_car/mot/cmd_vel avec un buffer de 100
    cmd_vel = nh.advertise<geometry_msgs::Twist>("auto_car/cmd_vel", 100);
    ROS_INFO("Complete.");

    ROS_INFO("Starting loop.");
    ros::spin();                                // Boucle de fonctionnement du package
}


// DÉFINITIONS DE FONCTIONS ======================================================================================
/*
description : Fonction callback appelé à chaque modification du topic /scan
paramètre : (const, sensor_msgs::LaserScan::ConstPtr, pointeur) scan_msg : message reçu.
*/
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    float lidar_pos = 0.0,                                                  // Position du lidar
            cmd_rot = 0.0;                                                  // Commande de braquage (en rad)
    geometry_msgs::Twist cmd;                                               // Commande torseur

    // Calcul du point moyen -------------------------------------------------------------------------------------
    for (int i = 0; i < scan_msg->ranges.size(); ++i) {                     // Pour chaque mesure
        if((i == left_angle_max) || (i == right_angle_max)) {               // Si l'angle correspond aux extrémités
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment; // calcul de l'angle associé
            lidar_pos += scan_msg->ranges[i] * std::sin(angle);             // Somme des mesures selon l'axe Y de la voiture (sin() car position du lidar)
        }
    }
    ROS_INFO("Lidar position: %f", lidar_pos);                              // Print sur le terminal ROS

    // Visualisation des points sur rviz -------------------------------------------------------------------------
    marker.header.frame_id = scan_msg->header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = lidar_pos;                                     // Position du nouveau point
    marker_pub.publish(marker);                                             // Publication du nouveau point

    // Interprétation des distances ------------------------------------------------------------------------------
    if (lidar_pos < (-middle_threshold)) {                                  // Si la distance est dans le seuil d'acceptation
        ROS_INFO("Turn right");
    } else if (lidar_pos > middle_threshold) {                              // Si la distance n'est pas dans le seuil d'acceptation et suppérieure
        ROS_INFO("Turn left");
    } else {                                                                // Sinon
        ROS_INFO("Lidar is in the middle");
    }
    if ((lidar_pos > (-middle_threshold)) || (lidar_pos < middle_threshold)) {
        cmd_rot = std::atan(lidar_pos/speed);                               // Rotation des roues (en rad)
        cmd.angular.z = cmd_rot * 180 / pi;                                 // Conversion en °
    } else {
        cmd.angular.z = 0.00;                                               // Rotation des roues (en °)
    }
    ROS_INFO("Ordre d'angle : %f", cmd.angular.z);
    cmd.linear.x = 10;                                                      // Vitesse de déplacement de la voiture (en %)
    cmd_vel.publish(cmd);                                                   // Publication sur le topic auto_car/cmd_vel
}

/*
description : Fonction callback appelé à chaque modification du topic /auto_car_ctrl/mot/vel
paramètre : (const, auto_car_ctrl::motors, pointeur) vel_msg : message reçu.
*/
void motCallback(const auto_car_ctrl::motors &vel_msg) {
    speed = vel_msg.vel.linear.x * d * pi / 156;                            // Calcul de la vitesse de la voiture (m/s)
}

/*
description : configure les paramètres des points à insérer sous RViz.
*/
void initRViz() {
    marker.ns = "lidar";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 90.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}

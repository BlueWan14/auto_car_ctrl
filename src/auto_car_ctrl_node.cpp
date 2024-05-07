#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <auto_car_ctrl/motors.h>
#include <visualization_msgs/Marker.h>
#include <unistd.h>

// DÉCLARATION DES VARIABLES =====================================================================================
#define car_size 0.09
#define dist_follow_wall 0.5

const float pi = std::acos(-1),                                                 // Calcul de pi
            left_edge_vision = pi / 2,
            right_edge_vision = - pi / 2;

float speed = 0.0;                                                              // Vitesse renvoyée par la carte Arduino

// Markers Rviz --------------------------------------------------------------------------------------------------
visualization_msgs::Marker lidar;
visualization_msgs::Marker sides;
visualization_msgs::Marker goal;
ros::Publisher lidar_pos_pub;
ros::Publisher sides_pub;
ros::Publisher goal_pub;

ros::Subscriber lidar_sub;                                                      // Subscriber pour le lidar
ros::Subscriber vel_sub;                                                        // Subscriber pour la Arduino
ros::Publisher cmd_vel;                                                         // Publisher pour la Arduino


// PROTOTYPES ====================================================================================================
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
void motCallback(const auto_car_ctrl::motors &vel_msg);


// MAIN ==========================================================================================================
int main(int argc, char** argv) {
    ROS_INFO("Starting car control...");

    ROS_INFO("Init ROS...");
    ros::init(argc, argv, "lidar_listener");                                    // Initalisation de ROS
    ROS_INFO("Complete.");

    ROS_INFO("Markers configuration...");
    // Marker "position lidar" -----------------------------------------------------------------------------------
    lidar.ns = "lidar";
    lidar.id = 0;
    lidar.type = visualization_msgs::Marker::SPHERE;
    lidar.action = visualization_msgs::Marker::ADD;
    lidar.pose.orientation.w = 1.0;
    lidar.scale.x = 0.2;
    lidar.scale.y = 0.2;
    lidar.scale.z = 0.2;
    lidar.color.a = 1.0;
    lidar.color.r = 0.62;
    lidar.color.b = 0.83;
    // Marker "too close from the side" --------------------------------------------------------------------------
    sides.ns = "tooClose";
    sides.id = 0;
    sides.type = visualization_msgs::Marker::POINTS;
    sides.action = visualization_msgs::Marker::ADD;
    sides.pose.orientation.w = 1.0;
    sides.scale.x = 0.05;
    sides.scale.y = 0.05;
    sides.scale.z = 0.05;
    sides.color.a = 1.0;
    sides.color.b = 1.0;
    // Marker "point le plus loin" -------------------------------------------------------------------------------
    goal.ns = "Objectif";
    goal.id = 0;
    goal.type = visualization_msgs::Marker::SPHERE;
    goal.action = visualization_msgs::Marker::ADD;
    goal.pose.orientation.w = 1.0;
    goal.scale.x = 0.1;
    goal.scale.y = 0.1;
    goal.scale.z = 0.1;
    goal.color.a = 1.0;
    ROS_INFO("Complete.");

    ROS_INFO("Subscribers and publishers creation...");
    ros::NodeHandle nh;                                                         // Communication ROS

    // Création des subscribers ----------------------------------------------------------------------------------
    lidar_sub = nh.subscribe("/scan", 100, &lidarCallback);
    vel_sub = nh.subscribe("/auto_car/mot/vel", 100, &motCallback);
    // Création des publishers -----------------------------------------------------------------------------------
    cmd_vel = nh.advertise<geometry_msgs::Twist>("auto_car/cmd_vel", 100);
    // Création des publishers pour Markers ----------------------------------------------------------------------
    lidar_pos_pub = nh.advertise<visualization_msgs::Marker>("auto_car/markers/lidar_pos", 100);
    sides_pub = nh.advertise<visualization_msgs::Marker>("auto_car/markers/sides", 100);
    goal_pub = nh.advertise<visualization_msgs::Marker>("auto_car/markers/goal", 100);
    ROS_INFO("Complete.");

    ROS_INFO("Starting loop.");
    ros::spin();                                                                // Boucle de fonctionnement du package
}

// DÉFINITIONS DE FONCTIONS ======================================================================================
/*
description : Fonction callback appelée à chaque modification du topic /scan
paramètre : (const, sensor_msgs::LaserScan::ConstPtr, pointeur) scan_msg : message reçu.
*/
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    geometry_msgs::Twist cmd;
    float range_max = 0.0,
        angle = 0.0, angle_range_max = 0.0, angle_tooClose = 0.0;
    bool tooClose = false;

    // Reset des markers
    geometry_msgs::Point p;
    sides.points.clear();
    goal.points.clear();

    goal.color.r = 0.0;
    goal.color.g = 1.0;
    goal.color.b = 0.0;

    for(int i = 0; i <= scan_msg->ranges.size(); ++i) {                         // Pour chaque distance mesurée
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;            // On calcul l'angle associé

        // Recherche de la distance la plus grande dans le champ de vision ---------------------------------------
        if(((angle < left_edge_vision) && (angle > right_edge_vision)) && (scan_msg->ranges[i] > range_max) && (scan_msg->ranges[i] < 12.0)) {
            range_max = scan_msg->ranges[i];
            angle_range_max = angle;
        }
        // Surveillance de la distance avec le mur le plus proche ------------------------------------------------
        if(((angle < left_edge_vision) && (angle > right_edge_vision)) && (scan_msg->ranges[i] < dist_follow_wall) && (scan_msg->ranges[i] > car_size)) {
            if (((angle_tooClose > angle) && (angle > 0)) || ((angle_tooClose < angle) && (angle < 0)) || (!tooClose))
                angle_tooClose = angle;
            // Configuration des markers
            p.x = scan_msg->ranges[i] * std::cos(angle);
            p.y = scan_msg->ranges[i] * std::sin(angle);
            sides.points.push_back(p);

            goal.color.r = 0.96;
            goal.color.g = 0.47;
            goal.color.b = 0.02;

            tooClose = true;
        }
    }
    if(tooClose) {
        cmd.angular.z = - angle_tooClose * 180 / pi;       // Commande de rotation des roues (en °)
        ROS_INFO("T'ES TROP PROCHE");
        ROS_INFO("%f", cmd.angular.z);
        ROS_INFO(" ---- ");
    } else {
        cmd.angular.z = angle_range_max * 180 / pi;      // Commande de rotation des roues (en °)
        ROS_INFO("%f", cmd.angular.z);
        ROS_INFO(" ---- ");
    }
    cmd.linear.x = 20;                                                          // Commande de vitesse du moteur (en %)
    cmd_vel.publish(cmd);                                                       // Publication sur le topic auto_car/cmd_vel

    // Publication des markers sur les topics --------------------------------------------------------------------
    lidar.header.frame_id = scan_msg->header.frame_id;
    lidar.header.stamp = ros::Time::now();
    lidar_pos_pub.publish(lidar); 

    sides.header.frame_id = scan_msg->header.frame_id;
    sides.header.stamp = ros::Time::now();
    sides_pub.publish(sides);

    goal.pose.position.x = range_max * std::cos(angle_range_max);
    goal.pose.position.y = range_max * std::sin(angle_range_max);
    goal.header.frame_id = scan_msg->header.frame_id;
    goal.header.stamp = ros::Time::now();
    goal_pub.publish(goal);
}

/*
description : Fonction callback appelé à chaque modification du topic /auto_car_ctrl/mot/vel
paramètre : (const, auto_car_ctrl::motors, pointeur) vel_msg : message reçu.
*/
void motCallback(const auto_car_ctrl::motors &vel_msg) {
    speed = vel_msg.vel.linear.x * 0.065 * pi / 156;                            // Reception de la vitesse mesurée (m/s)
}

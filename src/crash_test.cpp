#include <ros/ros.h>

// ROS messages libraries ----------------------------------------------------------------------------------------
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <auto_car_ctrl/motors.h>
#include <auto_car_ctrl/rosBool.h>


// DÉCLARATION DES VARIABLES =====================================================================================
#define car_size 0.09

const float pi = std::acos(-1),                                                 // Calcul de pi
            left_crash_vision = pi / 6,                                         // Définition du champ de vision pour considérer un crash
            right_crash_vision = - pi / 6;

float dist_follow_wall = 0.1,
      speed_max = 100.0;
bool rviz, isGoodWay;
visualization_msgs::Marker crash_marker;
auto_car_ctrl::rosBool crash;

// Other sub and publisher ---------------------------------------------------------------------------------------
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

    // Réupération des arguments du roslaunch --------------------------------------------------------------------
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
        // Définition du marker "position crash" -----------------------------------------------------------------
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
    ros::spin();                                                                // Lancement de la boucle ROS du noeud
}


// DÉFINITIONS DE FONCTIONS ======================================================================================
/*
description : Fonction callback appelée à chaque modification du topic "/auto_car/crash/goodway"
paramètre : (const, auto_car_ctrl::rosBool::ConstPtr, pointeur) goodWay_msg : le message ROS reçu.
*/
void goodWayCallBack(const auto_car_ctrl::rosBool::ConstPtr &goodWay_msg) {
    isGoodWay = goodWay_msg.answer;
}

/*
description : Fonction callback appelée à chaque modification du topic "/auto_car/arduino/mot"
paramètre : (const, auto_car_ctrl::motors::ConstPtr, pointeur) motor_msg : le message ROS reçu.
*/
void motorCallBack(const auto_car_ctrl::motors::ConstPtr &motor_msg) {
    if(crash.answer && ((motor_msg.rearObstacle < 10.0) || isGoodWay))          // Si un crash a été déclaré ET
                                                                                // qu'on détecte quelque chose derrière OU qu'on reprend la course
        crash.answer = false;                                                   // Alors on sort u mode "crash"
}

/*
description : Fonction callback appelée à chaque modification du topic "/scan"
paramètre : (const, sensor_msgs::LaserScan::ConstPtr, pointeur) scan_msg : le message ROS reçu.
*/
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    float angle = 0.0;
    geometry_msgs::Point p;

    // On supprime les points précédemment enregistrés -----------------------------------------------------------
    crash_marker.points.clear();

    // Définition du header pour le message ROS ------------------------------------------------------------------
    crash.header.frame_id = "crash";
    crash.header.stamp = ros::Time::now();
    
    // On analyse les mesures envoyées par le lidar --------------------------------------------------------------
    for(int i = 0; i <= scan_msg->ranges.size(); i++) {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;            // On calcul l'angle associé
        
        // Si l'angle est compris dans le champ de vision choisi et que la mesure est inférieure à la limite entre le mur et la voiture fixée par l'utilisateur
        if(((right_crash_vision < angle) && (angle < left_crash_vision)) && (car_size < scan_msg->ranges[i]) && (scan_msg->ranges[i] < dist_follow_wall)) {
            crash.answer = true;                                                // Alors on rentre dans le mode "crash"
            if(rviz) {                                                          // Si on a un affichage RVIZ
                p.x = scan_msg->ranges[i] * std::cos(angle);                    // On affiche tous les points
                p.y = scan_msg->ranges[i] * std::sin(angle);
                crash_marker.points.push_back(p);
            } else {                                                            // Sinon, on sort de la boucle for
                break;
            }
        }
    }

    // Publication des markers sur les topics si affichage RVIZ --------------------------------------------------
    if(rviz) {
        crash_marker.header.frame_id = scan_msg->header.frame_id;
        crash_marker.header.stamp = ros::Time::now();
        crash_marker_pub.publish(crash_marker);                                 // Publication du marker "auto_car/markers/crash"
    }

    crash_pub.publish(crash);                                                   // On publie le mode actuel ("crash"/"non crash")
                                                                                // sur le topic "auto_car/crash/iscrashed"
}

#include <ros/ros.h>
#include <auto_car_ctrl/rosBool.h>
#include <auto_car_ctrl/rosFloat.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <unistd.h>


// DÉCLARATION DES VARIABLES =====================================================================================
#define car_size 0.09

bool crash = false;
const float pi = std::acos(-1),                                                 // Calcul de pi
            left_edge_vision = pi / 2,
            right_edge_vision = - pi / 2;
float dist_follow_wall = 0.1,
      pourcent_range = 0.1;

// Markers Rviz --------------------------------------------------------------------------------------------------
visualization_msgs::Marker lidar;
visualization_msgs::Marker sides;
visualization_msgs::Marker goal_marker;
ros::Publisher lidar_pos_pub;
ros::Publisher sides_pub;
ros::Publisher goal_pub;

// Other sub and publisher ---------------------------------------------------------------------------------------
ros::Subscriber lidar_sub;
ros::Subscriber crash_sub;
ros::Publisher way_pub;
ros::Publisher processed_lidar;
ros::Publisher goodWay_sub;

// PROTOTYPES ====================================================================================================
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &);
void crashCallback(const auto_car_ctrl::rosBool &);

// MAIN ==========================================================================================================
int main(int argc, char** argv) {
    ROS_INFO("Starting car control...");

    ROS_INFO("Init ROS...");
    ros::init(argc, argv, "auto_car_ctrl");                                     // Initalisation de ROS
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
    goal_marker.ns = "Objectif";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.1;
    goal_marker.scale.y = 0.1;
    goal_marker.scale.z = 0.1;
    goal_marker.color.a = 1.0;
    goal_marker.color.g = 1.0;
    ROS_INFO("Complete.");

    ROS_INFO("Subscribers and publishers creation...");
    ros::NodeHandle nh;                                                         // Communication ROS
    ros::param::get("/dist_follow_wall", dist_follow_wall);
    ros::param::get("/pourcent_range", pourcent_range);

    // Création des subscribers ----------------------------------------------------------------------------------
    lidar_sub = nh.subscribe("/scan", 100, &lidarCallback);
    crash_sub = nh.subscribe("/auto_car/crash/iscrashed", 100, &crashCallback);
    // Création des publishers -----------------------------------------------------------------------------------
    processed_lidar = nh.advertise<auto_car_ctrl::rosFloat>("auto_car/cmd/lidar_process", 100);
    way_pub = nh.advertise<auto_car_ctrl::rosFloat>("auto_car/cmd/way", 100);
    goodWay_sub = nh.advertise<auto_car_ctrl::rosBool>("auto_car/crash/goodway", 100);
   
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
    std::list<float> delAngle;
    float accept_range, max_range,
        angle = 0.0, space = 0.0;
    bool skipAngle, WrongGoal;
    auto_car_ctrl::rosFloat goal, way;
    auto_car_ctrl::rosBool isGoodWay;
    geometry_msgs::Point p;

    goal.header.frame_id = "goal";
    goal.header.stamp = ros::Time::now();
    delAngle.clear();

    way.header.frame_id = "way";
    way.header.stamp = ros::Time::now();

    isGoodWay.header.frame_id = "isGoodWay";
    isGoodWay.header.stamp = ros::Time::now();

    // Reset des markers
    sides.points.clear();
    goal_marker.points.clear();
    
    while(delAngle.size() < scan_msg->ranges.size()) {
        WrongGoal = false;
        max_range = 0.0;

        for(int i = 0; i <= scan_msg->ranges.size(); i++) {                         // Pour chaque distance mesurée
            angle = scan_msg->angle_min + i * scan_msg->angle_increment;            // On calcul l'angle associé
            skipAngle = false;

            if(delAngle.size() != 0) {
                for (auto const &e: delAngle) {
                    if(angle == e) {
                        skipAngle = true;
                        break;
                    }
                }
            }
            // Recherche de la distance la plus grande dans le champ de vision ---------------------------------------
            if((!skipAngle) && (left_edge_vision > angle) && (angle > right_edge_vision) && (scan_msg->ranges[i] > max_range) && (scan_msg->ranges[i] < 12.0)) {
                max_range = scan_msg->ranges[i];                                    // Sauvegarde de la plus grande distance   
                goal.val = angle;                                                   // Angle associé à la distance la plus grande
            }
        }
            accept_range = max_range - (max_range * pourcent_range);
        for(int i = 0; i <= scan_msg->ranges.size(); i++) {
            angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            
            if((left_edge_vision > angle) && (angle > right_edge_vision) && (scan_msg->ranges[i] < accept_range) && (scan_msg->ranges[i] > dist_follow_wall)) {
                space = std::abs(scan_msg->ranges[i] * std::sin(angle - goal.val));
                if(space < dist_follow_wall) {
                    delAngle.push_back(goal.val);
                    WrongGoal = true;
                    p.x = scan_msg->ranges[i] * std::cos(angle);
                    p.y = scan_msg->ranges[i] * std::sin(angle);
                    sides.points.push_back(p);
                }
            }
        }
        if(!WrongGoal)
            break;
    }

    // Protocol en cas de crash ----------------------------------------------------------------------------------
    if(crash) {
        if((-0.52 > goal.val) && (goal.val > 0.52)) {   // Si le point le plus loin n'est pas dans un angle de 60° devant nous (0.52 = 30°)
            goal.val = - goal.val;                      // Alors on active le protocole recule
            way.val = -1;
            isGoodWay.answer = false;
        } else {                                        // Sinon, on désactive le protocole
            isGoodWay.answer = true;
        }
    } else {
        way.val = 1;                                    // Et on relance la marche avant
    }
    
    // Publication des valeurs sur les topics --------------------------------------------------------------------
    processed_lidar.publish(goal);
    way_pub.publish(way);
    goodWay_sub.publish(isGoodWay);

    // Publication des markers sur les topics --------------------------------------------------------------------
    lidar.header.frame_id = scan_msg->header.frame_id;
    lidar.header.stamp = ros::Time::now();
    lidar_pos_pub.publish(lidar);

    sides.header.frame_id = scan_msg->header.frame_id;
    sides.header.stamp = ros::Time::now();
    sides_pub.publish(sides);

    goal_marker.pose.position.x = max_range * std::cos(goal.val);
    goal_marker.pose.position.y = max_range * std::sin(goal.val);
    goal_marker.header.frame_id = scan_msg->header.frame_id;
    goal_marker.header.stamp = ros::Time::now();
    goal_pub.publish(goal_marker);
}

/*
description : Fonction callback appelée à chaque modification du topic /auto_car/crash
paramètre : (const, auto_car_ctrl::rosBool::ConstPtr, pointeur) crash_msg : message reçu.
*/
void crashCallback(const auto_car_ctrl::rosBool &crash_msg) {
    crash = crash_msg.answer;
}
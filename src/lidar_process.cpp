#include <ros/ros.h>
#include <auto_car_ctrl/rosBool.h>
#include <auto_car_ctrl/rosFloat.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <unistd.h>


// DÉCLARATION DES VARIABLES =====================================================================================
#define NBELT 100
#define car_size 0.9

bool crash = false,
     rviz = false;
const float pi = std::acos(-1),                                                 // Calcul de pi
            left_edge_vision = pi / 2,
            right_edge_vision = - pi / 2;
float dist_follow_wall = 0.15,
      pourcent_range = 0.05;

// Markers Rviz --------------------------------------------------------------------------------------------------
visualization_msgs::Marker lidar;
visualization_msgs::Marker sides;
visualization_msgs::Marker goal_marker;
visualization_msgs::Marker wrongGoal_marker;
ros::Publisher lidar_pos_pub;
ros::Publisher sides_pub;
ros::Publisher goal_pub;
ros::Publisher wrongGoal_pub;

// Other sub and publisher ---------------------------------------------------------------------------------------
ros::Subscriber lidar_sub;
ros::Subscriber crash_sub;
ros::Publisher way_pub;
ros::Publisher processed_lidar;
ros::Publisher goodWay_pub;

// PROTOTYPES ====================================================================================================
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &);
void crashCallback(const auto_car_ctrl::rosBool &);

// MAIN ==========================================================================================================
int main(int argc, char** argv) {
    ROS_INFO("Starting car control...");

    ROS_INFO("Init ROS...");
    ros::init(argc, argv, "auto_car_ctrl");                                     // Initalisation de ROS
    ros::param::get("/dist_follow_wall", dist_follow_wall);
    ros::param::get("/pourcent_range", pourcent_range);
    ros::param::get("/rviz", rviz);
    ROS_INFO("Complete.");

    if(rviz) {
        ROS_INFO("Markers configuration...");
        // Marker "position lidar" -----------------------------------------------------------------------------------
        lidar.ns = "lidar";
        lidar.id = 0;
        lidar.type = visualization_msgs::Marker::ARROW;
        lidar.action = visualization_msgs::Marker::ADD;
        lidar.scale.x = 0.2;
        lidar.scale.y = 0.05;
        lidar.scale.z = 0.05;
        lidar.color.a = 1.0;
        lidar.color.r = 0.62;
        lidar.color.b = 0.83;
        // Marker "too close from the side" --------------------------------------------------------------------------
        sides.ns = "tooClose";
        sides.id = 0;
        sides.type = visualization_msgs::Marker::POINTS;
        sides.action = visualization_msgs::Marker::ADD;
        sides.pose.orientation.w = 0.0;
        sides.scale.x = 0.03;
        sides.scale.y = 0.03;
        sides.scale.z = 0.03;
        sides.color.a = 1.0;
        sides.color.b = 1.0;
        // Marker "point le plus loin" -------------------------------------------------------------------------------
        goal_marker.ns = "Objectif";
        goal_marker.id = 0;
        goal_marker.type = visualization_msgs::Marker::LINE_STRIP;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.orientation.w = 0.0;
        goal_marker.scale.x = 0.01;
        goal_marker.scale.y = 0.01;
        goal_marker.scale.z = 0.01;
        goal_marker.color.a = 1.0;
        goal_marker.color.g = 1.0;
        ROS_INFO("Complete.");
        // Marker "mauvais point le plus loin" -------------------------------------------------------------------------------
        wrongGoal_marker.ns = "WrongObjectif";
        wrongGoal_marker.id = 0;
        wrongGoal_marker.type = visualization_msgs::Marker::POINTS;
        wrongGoal_marker.action = visualization_msgs::Marker::ADD;
        wrongGoal_marker.pose.orientation.w = 0.0;
        wrongGoal_marker.scale.x = 0.02;
        wrongGoal_marker.scale.y = 0.02;
        wrongGoal_marker.scale.z = 0.02;
        wrongGoal_marker.color.a = 1.0;
        wrongGoal_marker.color.r = 1.00;
        wrongGoal_marker.color.g = 0.69;
        ROS_INFO("Complete.");
    }

    ROS_INFO("Subscribers and publishers creation...");
    ros::NodeHandle nh;                                                         // Communication ROS

    // Création des subscribers ----------------------------------------------------------------------------------
    lidar_sub = nh.subscribe("/scan", 100, &lidarCallback);
    crash_sub = nh.subscribe("/auto_car/crash/iscrashed", 100, &crashCallback);
    // Création des publishers -----------------------------------------------------------------------------------
    processed_lidar = nh.advertise<auto_car_ctrl::rosFloat>("auto_car/cmd/lidar_process", 100);
    way_pub = nh.advertise<auto_car_ctrl::rosFloat>("auto_car/cmd/way", 100);
    goodWay_pub = nh.advertise<auto_car_ctrl::rosBool>("auto_car/crash/goodway", 100);
   
    // Création des publishers pour Markers ----------------------------------------------------------------------
    if(rviz) {
        lidar_pos_pub = nh.advertise<visualization_msgs::Marker>("auto_car/markers/lidar_pos", 100);
        sides_pub = nh.advertise<visualization_msgs::Marker>("auto_car/markers/sides", 100);
        goal_pub = nh.advertise<visualization_msgs::Marker>("auto_car/markers/goal", 100);
        wrongGoal_pub = nh.advertise<visualization_msgs::Marker>("auto_car/markers/wrongGoal", 100);
    }
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
    float delAngle[NBELT];
    float accept_range, max_range,
        angle = 0.0, space = 0.0;
    bool skipAngle, WrongGoal;
    int size = 0;
    auto_car_ctrl::rosFloat goal, way;
    auto_car_ctrl::rosBool isGoodWay;
    geometry_msgs::Point p;

    goal.header.frame_id = "goal";
    goal.header.stamp = ros::Time::now();

    way.header.frame_id = "way";
    way.header.stamp = ros::Time::now();

    isGoodWay.header.frame_id = "isGoodWay";
    isGoodWay.header.stamp = ros::Time::now();

    // Reset des markers
    sides.points.clear();
    goal_marker.points.clear();
    goal_marker.points.push_back(p);
    wrongGoal_marker.points.clear();
    
    for(int size=0; size < NBELT; size++) {
        WrongGoal = false;
        max_range = 0.0;

        for(int i = 0; i <= scan_msg->ranges.size(); i++) {                         // Pour chaque distance mesurée
            angle = scan_msg->angle_min + i * scan_msg->angle_increment;            // On calcul l'angle associé
            skipAngle = false;

            if((left_edge_vision > angle) && (angle > right_edge_vision) && (car_size < scan_msg->ranges[i]) && (scan_msg->ranges[i] < scan_msg->range_max)) {
                if(size != 0) {
                    for(int i=0; i < size; i++) {
                        if(angle == delAngle[i]) {
                            skipAngle = true;
                            break;
                        }
                    }
                }
                // Recherche de la distance la plus grande dans le champ de vision ---------------------------------------
                if((!skipAngle) && (scan_msg->ranges[i] > max_range)) {
                    max_range = scan_msg->ranges[i];                                    // Sauvegarde de la plus grande distance   
                    goal.val = angle;                                                   // Angle associé à la distance la plus grande
                }
            }
        }
        accept_range = max_range - (max_range * pourcent_range);
        for(int i = 0; i <= scan_msg->ranges.size(); i++) {
            angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            
            if((left_edge_vision > angle) && (angle > right_edge_vision) && (0.0 < scan_msg->ranges[i]) && (scan_msg->ranges[i] < accept_range)) {
                space = std::abs(scan_msg->ranges[i] * std::sin(angle - goal.val));
                if(space < dist_follow_wall) {
                    delAngle[size] = goal.val;
                    WrongGoal = true;
                    if(rviz) {
                        // sides markers
                        p.x = scan_msg->ranges[i] * std::cos(angle);
                        p.y = scan_msg->ranges[i] * std::sin(angle);
                        sides.points.push_back(p);

                        // wrong goals marker
                        p.x = max_range * std::cos(goal.val);
                        p.y = max_range * std::sin(goal.val);
                        wrongGoal_marker.points.push_back(p);
                    }
                }
            } else {
                break;
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
    goodWay_pub.publish(isGoodWay);

    // Publication des markers sur les topics --------------------------------------------------------------------
    if(rviz) {
        lidar.header.frame_id = scan_msg->header.frame_id;
        lidar.header.stamp = ros::Time::now();
        lidar.pose.orientation.z = std::sin(goal.val / 2);
        lidar.pose.orientation.w = std::cos(goal.val / 2);
        lidar_pos_pub.publish(lidar);

        sides.header.frame_id = scan_msg->header.frame_id;
        sides.header.stamp = ros::Time::now();
        sides_pub.publish(sides);

        p.x = max_range * std::cos(goal.val);
        p.y = max_range * std::sin(goal.val);
        goal_marker.points.push_back(p);
        goal_marker.header.frame_id = scan_msg->header.frame_id;
        goal_marker.header.stamp = ros::Time::now();
        goal_pub.publish(goal_marker);

        wrongGoal_marker.header.frame_id = scan_msg->header.frame_id;
        wrongGoal_marker.header.stamp = ros::Time::now();
        wrongGoal_pub.publish(wrongGoal_marker);
    }
}

/*
description : Fonction callback appelée à chaque modification du topic /auto_car/crash
paramètre : (const, auto_car_ctrl::rosBool::ConstPtr, pointeur) crash_msg : message reçu.
*/
void crashCallback(const auto_car_ctrl::rosBool &crash_msg) {
    crash = crash_msg.answer;
}
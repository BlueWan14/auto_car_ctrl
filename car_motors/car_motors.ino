#define nullptr NULL

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <auto_car_ctrl/motors.h>


// PINS ================================================================================
// Servo moteurs
#define pin_cod 2
#define Echo 4
#define Trigger 5
#define servo_vel 6
#define servo_dir 7
#define pin_SW 10

// BORNES DE COMMANDES =================================================================
// Bornes liées à la vitesse
#define ForwardMax 2000
#define NEUTRAL 1500
#define BackwardMax 1000
// Bornes liées à la direction de la voiture
#define MIDDLE 96
// Spécifications télécommandes
#define auto_limit 1800


// VARIABLES ===========================================================================
// Initialisation des variables servomoteurs
Servo MotorLinear;
Servo MotorAngular;
// Bornes liées à la direction de la voiture
int TurnLeftMax,
    TurnRightMax;
// Initialisation des variables de commande
int MotSpeed = NEUTRAL,
    MotAngle = MIDDLE,
    Speed_mem = NEUTRAL;
// Initialisation des variables de télécommande
int SW = 0;


// DÉFINITIONS DE FONCTIONS ============================================================
/*
description : Fonction callback appelé à chaque modification du topic auto_car/mot/cmd_vel
paramètre : (const, geometry_msgs::Twist, pointeur) msg : message reçu.
*/
void cmdCallback(const geometry_msgs::Twist &msg) {
  if(SW > auto_limit) {
    // Lecture des commandes selon le torseur envoyé -------------------------------------
    float RosSpeedX = msg.linear.x;
    float RosAngleZ = msg.angular.z;
    
    // Calcul des commandes réelles ------------------------------------------------------
    MotSpeed = PtV(RosSpeedX, ForwardMax, NEUTRAL, BackwardMax);
    MotAngle = RosAngleZ + MIDDLE;

    // Les variables sont contraintes sur leur plage de fonctionnement, elle ne peuvent pas en sortir
    MotAngle = constrain(MotAngle, TurnRightMax, TurnLeftMax);

    // Pour passer en vitesse négative, il faut d'abord "freiner" en engageant une première marche arrière
    break_sys();

    // Écriture de la nouvelle commande --------------------------------------------------
    MotorLinear.writeMicroseconds(MotSpeed);
    MotorAngular.write(MotAngle);
  }
}

/*
description : diminutif de "pourcentage to value". Calcul la valeur réelle selon le pourcentage.
paramètre : (float) pourcent : pourcentage.
            (int) Max : valeur maximal du résultat
            (int) Null : offset
            (int) Min : valeur minimal du résultat
return : (int) val : entier.
*/
int PtV(float pourcent, int Max, int Null, int Min) {
  int val;

  if(pourcent >= 0) {
    pourcent = pourcent * (Max - Null) / 100.0;
  } else if(pourcent < 0) {
    pourcent = pourcent * (Null - Min) / 100.0;
  }
  val = (int)pourcent + Null;

  return val;
}

/*
description : diminutif de "value to pourcentage". Calcul le pourcentage selon la valeur réelle.
paramètre : (int) val : entier
            (int) Max : valeur maximal du résultat
            (int) Null : offset
            (int) Min : valeur minimal du résultat
return : (float) pourcent : pourcentage.
*/
float VtP(int val, int Max, int Null, int Min) {
  float pourcent;

  val = val - Null;
  if(val >= 0) {
    pourcent = (float)val / (Max - Null) * 100.0;
  } else if(val < 0) {
    pourcent = (float)val / (Null - Min) * 100.0;
  }

  return pourcent;
}

/*
description : mise au point mort du moteur.
*/
void break_sys() {
  if((MotSpeed < NEUTRAL) && (Speed_mem >= NEUTRAL)) {
    MotorLinear.writeMicroseconds(BackwardMax); // Freinage
    delay(50);                                  // Tempo pour changer l'écriture 
    MotorLinear.writeMicroseconds(NEUTRAL);     // Remise au point mort
    delay(115);                                 // Tempo minimale pour relancer le moteur
  }
  return;
}


// COMMUNICATION ROS ===================================================================
// Initialisation des variables liées à ROS
ros::NodeHandle nh;
auto_car_ctrl::motors msg_mot;
// Initialisation d'export de donnée sur le topic "auto_car/mot/vel"
ros::Publisher return_vel("auto_car/arduino/mot", &msg_mot);
// Initialisation de reception de donnée sur le topic "auto_car/cmd_vel"
ros::Subscriber<geometry_msgs::Twist> cmd_vel("auto_car/arduino/cmd_vel", &cmdCallback);

// SETUP ===============================================================================
void setup() {
  // Définition des pins ---------------------------------------------------------------
  pinMode(pin_cod, INPUT);
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(pin_SW, INPUT);

  // Association des pins avec les servomoteurs ----------------------------------------
  MotorLinear.attach(servo_vel);
  MotorAngular.attach(servo_dir);

  // Lancement des communications ROS --------------------------------------------------
  nh.initNode();
  nh.subscribe(cmd_vel);      // On s'abonne à cmd_vel
  nh.advertise(return_vel);   // On configure return_vel afin de publier dessus

  TurnLeftMax = MIDDLE + 40;
  TurnRightMax = MIDDLE - 40;

  // Initialisation des moteurs --------------------------------------------------------
  MotorLinear.writeMicroseconds(NEUTRAL);
  MotorAngular.write(MIDDLE);

  while (!nh.connected())
    nh.spinOnce();
  if(nh.getParam("/angle_max_left", &TurnLeftMax))
    TurnLeftMax = TurnLeftMax + MIDDLE;
  if(nh.getParam("/angle_max_right", &TurnRightMax))
    TurnRightMax = TurnRightMax + MIDDLE;
}

// MAIN ================================================================================
void loop() {
  // Réception des données de la télécommande ------------------------------------------
  SW = pulseIn(pin_SW, HIGH); // Commande auto/manuel

  if(SW <= auto_limit) {
    MotSpeed = NEUTRAL;
    MotAngle = MIDDLE;

    // Écriture de la nouvelle commande
    MotorLinear.writeMicroseconds(MotSpeed);
    MotorAngular.write(MotAngle);
  }

  digitalWrite(Trigger, LOW); // On efface l'etat logique de TRIG
  delayMicroseconds(2);
  digitalWrite(Trigger, HIGH); // On met la broche TRIG a "1" pendant 10µS
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW); // On remet la broche TRIG a "0"
  msg_mot.rearObstacle = pulseIn(Echo, HIGH) * 0.034 / 2;

  // Écriture de la vitesse et la rotation actuelle sur return_vel ---------------------
  msg_mot.vel.linear.x = VtP(MotSpeed, ForwardMax, NEUTRAL, BackwardMax);
  msg_mot.vel.angular.z = MotAngle - MIDDLE;
  return_vel.publish(&msg_mot);

  // Lecture de la communication avec ROS ----------------------------------------------
  nh.spinOnce();
  delay(115);
}

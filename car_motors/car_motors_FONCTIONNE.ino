#define nullptr NULL

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <auto_car_ctrl/motors.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


// PINS ================================================================================
// Servo moteurs
#define servo_vel 6
#define servo_dir 7
#define pin_dir 8
#define pin_speed 9
#define pin_SW 10
#define pin_Batt A0

// BORNES DE COMMANDES =================================================================
// Bornes liées à la vitesse
#define ForwardMax 2000
#define NEUTRAL 1500
#define BackwardMax 1000
// Bornes liées à la direction de la voiture
#define MIDDLE 90
// Bornes liées à la vitesse
#define ctrl_forward_max 1775
#define ctrl_neutral 1360
#define ctrl_backward_max 905
// Bornes liées au braquage de la voiture
#define ctrl_left_max 1530
#define ctrl_middle 1306
#define ctrl_right_max 1078
// Spécifications télécommandes
#define auto_limit 1800
#define treshold_ctrller 30
// Valeurs de tension batterie
#define Tension_Max_Batterie 8.2
#define Voltage_Alert_Batt 6.5
#define Tension_Min_Batterie 6.0


// VARIABLES ===========================================================================
// Initialisation des variables servomoteurs
Servo MotorLinear;
Servo MotorAngular;
// Bornes liées à la direction de la voiture
int TurnLeftMax = 115,
      TurnRightMax = 65;
// Initialisation des variables de commande
int MotSpeed = NEUTRAL,
    MotAngle = MIDDLE,
    Speed_mem = NEUTRAL;
// Initialisation des variables de télécommande
int SW = 0,
    average_speed = ctrl_neutral,
    average_dir = ctrl_middle,
    i_speed = 1,
    i_dir = 1;
// Initialisation des variables de mesure de tension
int Pourcentage_Batt;
unsigned long MillisMem;
float Tension;
LiquidCrystal_I2C lcd(0x27,16,2);


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

/*
description : lit et traite les informations envoyées par la télécommande
paramètre : (int) pin : pin recevant les données
            (int) defaultValue : valeur neutre de la donnée
return : (int) Commande.
*/
int readController(int pin, int defaultValue, int* average, int* i) {
  int cmd = pulseIn(pin, HIGH);

  if(((cmd < (defaultValue - treshold_ctrller)) || (cmd > (defaultValue + treshold_ctrller))) && (cmd != 0)) {
    return cmd;
  } else {
    return defaultValue;
  }
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
  pinMode(pin_speed, INPUT);
  pinMode(pin_dir, INPUT);
  pinMode(pin_SW, INPUT);

  // Initialisation du lcd -------------------------------------------------------------
  lcd.init();                      
  lcd.backlight();

  // Association des pins avec les servomoteurs ----------------------------------------
  MotorLinear.attach(servo_vel);
  MotorAngular.attach(servo_dir);

  // Lancement des communications ROS --------------------------------------------------
  nh.initNode();
  nh.subscribe(cmd_vel);      // On s'abonne à cmd_vel
  nh.advertise(return_vel);   // On configure return_vel afin de publier dessus

  if(nh.getParam("/angle_max_left", &TurnLeftMax))
    TurnLeftMax = TurnLeftMax + MIDDLE;
  if(nh.getParam("/angle_max_right", &TurnRightMax))
    TurnRightMax = TurnRightMax + MIDDLE;

  // Initialisation des moteurs --------------------------------------------------------
  MotorLinear.writeMicroseconds(NEUTRAL);
  MotorAngular.write(MIDDLE);

  MillisMem = millis();
}

// MAIN ================================================================================
void loop() {
  // Réception des données de la télécommande ------------------------------------------
  SW = pulseIn(pin_SW, HIGH); // Commande auto/manuel

  if(SW <= auto_limit) {
    
    // Calcul des commandes réelles
    MotSpeed = constrain(PtV(VtP(readController(pin_speed, ctrl_neutral, &average_speed, &i_speed), ctrl_forward_max+treshold_ctrller, ctrl_neutral, ctrl_backward_max-treshold_ctrller), ForwardMax, NEUTRAL, BackwardMax), BackwardMax, ForwardMax);
    MotAngle = constrain(PtV(VtP(readController(pin_dir, ctrl_middle, &average_dir, &i_dir), ctrl_left_max+treshold_ctrller, ctrl_middle, ctrl_right_max-treshold_ctrller), TurnLeftMax, MIDDLE, TurnRightMax), TurnRightMax, TurnLeftMax);

    // Pour passer en vitesse négative, il faut d'abord "freiner" en engageant une première marche arrière
    break_sys();

    // Écriture de la nouvelle commande
    MotorLinear.writeMicroseconds(MotSpeed);
    MotorAngular.write(MotAngle);
  }

  if((millis() - MillisMem) > 1000) {
    // Calcul et affichage de la tension batterie ----------------------------------------
    Tension = analogRead(pin_Batt) * 4.8 / 1023 * 2;
    Tension = constrain(Tension, Tension_Min_Batterie, Tension_Max_Batterie);
    Pourcentage_Batt = (Tension - Tension_Min_Batterie) * 100 / (Tension_Max_Batterie - Tension_Min_Batterie);

    // Affichage de la valeur sur le lcd -------------------------------------------------
    lcd.clear();
    lcd.setCursor(0,0);  
    lcd.print("Batterie = " + String(Pourcentage_Batt) + "%");
    if (Tension < Voltage_Alert_Batt) {
      lcd.setCursor(6,1);
      lcd.print("HELP");
    }
    msg_mot.voltage = Tension;
  }

  // Écriture de la vitesse et la rotation actuelle sur return_vel ---------------------
  msg_mot.vel.linear.x = VtP(MotSpeed, ForwardMax, NEUTRAL, BackwardMax);
  msg_mot.vel.angular.z = MotAngle - MIDDLE;
  return_vel.publish(&msg_mot);

  // Lecture de la communication avec ROS ----------------------------------------------
  nh.spinOnce();
  delay(115);
}
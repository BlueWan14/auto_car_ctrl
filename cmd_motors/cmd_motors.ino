#define nullptr NULL

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <auto_car_ctrl/motors.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


// PROTOTYPES ==========================================================================
void cmdCallback(const geometry_msgs::Twist &msg);
int PtV(float, int, int, int);
float VtP(int, int, int, int);
void break_sys();


// PINS ================================================================================
// RC controller
#define pin_dir 8
#define pin_vel 9
#define pin_SW 10
// Servo moteurs
#define servo_vel 6
#define servo_dir 7
// Codeur vitesse
#define Coder 2
// Tension batterie
#define Bat A0

// BORNES DE COMMANDES =================================================================
// Bornes liées à la vitesse
#define ForwardMax 3000
#define NEUTRAL 2000
#define BackwardMax 1000
// Bornes des PWM de commande de vitesse
#define ctrl_forward_max 1530
#define ctrl_neutral 1304
#define ctrl_backward_max 1078
// Bornes liées au braquage de la voiture
#define TurnLeftMax 115
#define MIDDLE 90
#define TurnRightMax 70
// Bornes des PWM de commande de direction
#define ctrl_left_max 1809
#define ctrl_middle 1367
#define ctrl_right_max 902

// Sensibilité de la manette
#define treshold_ctrller 5

// COMMUNICATION ROS ===================================================================
// Initialisation des variables liées à ROS
ros::NodeHandle nh; 
auto_car_ctrl::motors msg_mot;

// Initialisation d'export de donnée sur le topic "speed"
ros::Publisher return_vel("auto_car/mot/vel", &msg_mot);
// Initialisation de reception de donnée sur le topic "turtle1/cmd_vel"
ros::Subscriber<geometry_msgs::Twist> cmd_vel("auto_car/cmd_vel", &cmdCallback); 

// VARIABLES ==========================================================================
// Initialisation des variables servomoteurs
Servo MotorLinear;
Servo MotorAngular;
// Communication avec le LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Initialisation des variables de commande
int MotSpeed,
  average_cmd_speed = ctrl_neutral, average_cmd_dir = ctrl_middle,
  i_speed = 1,                      i_dir = 1,
  i_timer = 0;
float MotAngle,
  speed,                            dir;
unsigned long cmd_dir, cmd_speed, cmd_SW;
volatile long pulseCount = 0;
volatile unsigned long elapsedTime = 0;
volatile float rotationSpeed = 0.0;


// SETUP ===============================================================================
void setup() {
  // Association des pins avec la manette
  pinMode(pin_dir, INPUT);
  pinMode(pin_vel, INPUT);
  pinMode(pin_SW, INPUT);

  // Association des pins avec les servomoteurs ----------------------------------------
  MotorLinear.attach(servo_vel);
  MotorAngular.attach(servo_dir);

  // Association de pins avec l'asservissement vitesse ---------------------------------
  pinMode(Coder, INPUT);
  attachInterrupt(0, countPulse, CHANGE); // Utilise une interruption sur la broche A 
  cli();

  // Lancement des communications ROS --------------------------------------------------
  nh.initNode();
  nh.subscribe(cmd_vel);      // On s'abonne à cmd_vel
  nh.advertise(return_vel);   // On configure return_vel afin de publier dessus

  // Initialisation du LCD -------------------------------------------------------------
  lcd.init();
  lcd.backlight();

  // Mise à 0 des moteurs --------------------------------------------------------------
  MotorLinear.writeMicroseconds(NEUTRAL);
  MotorAngular.write(MIDDLE);

  // Initialisation du timer interne pour l'asservissement -----------------------------
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0));
  TCCR1A &= ~((1<<COM1B1) | (1<<COM1B0));
  TCCR1A &= ~((1<<WGM11) | (1<<WGM10));
  TCCR1B &= ~(1<<WGM12);
  TCCR1B |= (1<<WGM13);
  TCCR1B |=  (1<<CS10);
  TCCR1B &= ~((1<<CS12) | (1<<CS11));
  ICR1=40000;
  TIMSK1 |= (1<< TOIE1);
  sei();

  // Renvoie des extrêmums sur ROS -----------------------------------------------------
  msg_mot.rot_right_max = TurnRightMax;
  msg_mot.rot_left_max = TurnLeftMax;
}

// MAIN ================================================================================
void loop() {
  // Écriture de la vitesse et la rotation actuelle sur return_vel ---------------------
  msg_mot.vel.linear.x = rotationSpeed;
  msg_mot.vel.angular.z = MotAngle - MIDDLE;
  return_vel.publish(&msg_mot);

  cmd_SW = pulseIn(pin_SW, HIGH);

  if(cmd_SW < 1700) {
    // Lecture des commandes de la manette (en %) ----------------------------------------
    cmd_dir = pulseIn(pin_dir, HIGH);
    cmd_speed = pulseIn(pin_vel, HIGH);

    average_cmd_dir += cmd_dir;
    i_dir++;
    average_cmd_speed += cmd_speed;
    i_speed++;

    if(((average_cmd_dir / i_dir) < (ctrl_middle - treshold_ctrller)) || ((average_cmd_dir / i_dir) > (ctrl_middle + treshold_ctrller))) {
      dir = VtP(cmd_dir, ctrl_left_max, ctrl_middle, ctrl_right_max);
      average_cmd_dir = ctrl_middle;
      i_dir = 1;
    } else {
      dir = 0;
    }
    if(((average_cmd_speed / i_speed) < (ctrl_neutral - treshold_ctrller)) || ((average_cmd_speed / i_speed) > (ctrl_neutral + treshold_ctrller))) {
      speed = VtP(cmd_speed, ctrl_forward_max, ctrl_neutral, ctrl_backward_max);
      average_cmd_speed = ctrl_neutral;
      i_speed = 1;
    } else {
      speed = 0;
    }

    // Calcul des commandes réelles ------------------------------------------------------
    MotSpeed = PtV(speed, ForwardMax, NEUTRAL, BackwardMax);
    MotAngle = PtV(dir, TurnLeftMax, MIDDLE, TurnRightMax);

    // Pour passer en vitesse négative, il faut d'abord "freiner" en engageant une première marche arrière
    break_sys();

    // Écriture de la nouvelle commande --------------------------------------------------
    MotorLinear.writeMicroseconds(MotSpeed);
    MotorAngular.write(MotAngle);
  }

  // Lecture de la communication avec ROS ----------------------------------------------
  nh.spinOnce();
  delay(10);
}


// DÉFINITIONS DE FONCTIONS ============================================================
/*
description : Fonction callback appelé à chaque modification du topic auto_car/mot/cmd_vel
paramètre : (const, geometry_msgs::Twist, pointeur) msg : message reçu.
*/
void cmdCallback(const geometry_msgs::Twist &msg) {
  if(cmd_SW > 1700) {
    // Sauvegarde de la vitesse en mémoire -----------------------------------------------
    int Speed_mem = MotSpeed;

    // Lecture des commandes selon le torseur envoyé -------------------------------------
    float RosSpeedX = msg.linear.x;
    float RosAngleZ = msg.angular.z;
    
    // Calcul des commandes réelles ------------------------------------------------------
    MotSpeed = PtV(RosSpeedX, ForwardMax, NEUTRAL, BackwardMax);
    MotAngle = RosAngleZ + MIDDLE;

    // Les variables sont contraintes sur leur plage de fonctionnement, elle ne peuvent pas en sortir
    MotAngle = constrain(MotAngle, TurnRightMax, TurnLeftMax);

    // Pour passer en vitesse négative, il faut d'abord "freiner" en engageant une première marche arrière
    if((MotSpeed < NEUTRAL) && (Speed_mem >= NEUTRAL))
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

  if(pourcent > 0) {
    pourcent = pourcent * (Max - Null) / 100.0;
  } else if(pourcent < 0) {
    pourcent = pourcent * (Null - Min) / 100.0;
  }
  val = (int)(pourcent + Null);

  return val;
}

/*
description : diminutif de "value to pourcentage". Calcul le pourcentage selon la valeur réelle.
paramètre : (int) val : entier.
            (int) Max : valeur maximal du résultat
            (int) Null : offset
            (int) Min : valeur minimal du résultat
return : (float) pourcent : pourcentage.
*/
float VtP(int val, int Max, int Null, int Min) {
  float pourcent;

  val = val - Null;
  if(val > 0) {
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
  MotorLinear.writeMicroseconds(BackwardMax); // Freinage
  delay(50);                                  // Tempo pour changer l'écriture 
  MotorLinear.writeMicroseconds(NEUTRAL);     // Remise au point mort
  delay(115);                                 // Tempo minimale pour relancer le moteur
  return;
}


// DÉFINITIONS D'INTERRUPTION ============================================================
/*
description : Fonction interruption appelé périodiquement par le timer interne "TIMER1_OVF_vect".
              Soit toutes les 5 ms.
*/
ISR(TIMER1_OVF_vect) {
  i_timer++;
  if (i_timer>=10){                                             // Toutes les 50ms
    rotationSpeed = ((pulseCount*20.0)/(360.0))*(22.0/70.0)*60; // On calcul les tr/min en fonction du nombre d'impulsions envoyées par le codeur
    // Réinitialisation du compteur d'impulsion et du timer
    pulseCount = 0;
    i_timer = 0;
  }
}

/*
description : Fonction interruption appelé à chaque impulsion envoyée par le codeur.
              Elle ne fait qu'incrémenter un compteur.
*/
void countPulse(){
  pulseCount++; 
}

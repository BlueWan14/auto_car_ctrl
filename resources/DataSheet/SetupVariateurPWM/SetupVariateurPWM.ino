#include <Servo.h>


// 0 < val < 20000
#define NEUTRAL 2000
#define MAX 3000
#define MIN 1000

Servo myservo, servo;
String sens;


void setup() {
  Serial.begin(9600);
  myservo.attach(6);
  myservo.writeMicroseconds(NEUTRAL);
}

void loop() {
  if(Serial.available() > 0) {
    //Lire le mot écrit puis enlever tout \r \n et espace à la fin du mot
    sens = Serial.readString(); 
    sens.trim();
  }

  if(sens == "0") {
    Serial.println("Fin enregistrement");
    myservo.writeMicroseconds(MIN);
    myservo.writeMicroseconds(NEUTRAL);
  } else if(sens == "1") {
    Serial.println("Enregistement Vmax");
    myservo.writeMicroseconds(MAX);
  } else if(sens == "2") {
    Serial.println("Enregistement Vmin");
    myservo.writeMicroseconds(MIN);
  }

  delay(1000);
}


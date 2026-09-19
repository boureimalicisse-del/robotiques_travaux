#include "MeAuriga.h"

#define LEDNUM 12
#define LEDPIN 44
MeRGBLed led(PORT0, LEDNUM);

const int NOMBRE_DE_LED_A_ACTIVER = 7;
const int ledsDevant[NOMBRE_DE_LED_A_ACTIVER] = { 0, 1, 2, 3, 4, 5, 11 };
const int ledsArriere[NOMBRE_DE_LED_A_ACTIVER] = { 11, 10, 9, 8, 7, 6, 5 };

MeUltrasonicSensor capteurDistance(PORT_10);

enum Etat {
  ARRET,
  MARCHE,
  LENT,
  RECULE,
  PIVOTE
};
Etat currentState = MARCHE;

unsigned long stateMillis = 0;

const unsigned long DUREE_ARRET = 2000;
const unsigned long DUREE_RECULE = 1000;
const unsigned long DUREE_PIVOTE = 800;

const int DISTANCE_ACTIVATION = 100;
const int DISTANCE_ARRET = 30;

const int VITESSE_CROISIERE_POURCENT = 60;
const int VITESSE_LENTE_POURCENT = 50;
const int PWM_MAX = 255;

const int M1_PWM = 11;
const int M1_IN1 = 48;
const int M1_IN2 = 49;

const int M2_PWM = 10;
const int M2_IN1 = 47;
const int M2_IN2 = 46;

void setup() {
  Serial.begin(9600);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_IN1, OUTPUT);

  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);

  led.setpin(LEDPIN);
}

void loop() {
  unsigned long currentMillis = millis();
  int distance = lireDistance(currentMillis);

  gererEtats(currentMillis, distance);
}

void gererEtats(unsigned long currentMillis, int distance) {
  switch (currentState) {
    case MARCHE:
      gererMarche();
      if (distance < DISTANCE_ACTIVATION) {
        currentState = LENT;
      }
      break;

    case LENT:
      gererLent();
      if (distance >= DISTANCE_ACTIVATION) {
        currentState = MARCHE;
      } else if (distance < DISTANCE_ARRET) {
        currentState = ARRET;
        stateMillis = currentMillis;
      }
      break;

    case ARRET:
      gererArret();
      if (currentMillis - stateMillis >= DUREE_ARRET) {
        currentState = RECULE;
        stateMillis = currentMillis;
      }
      break;

    case RECULE:
      gererRecule();
      if (currentMillis - stateMillis >= DUREE_RECULE) {
        currentState = PIVOTE;
        stateMillis = currentMillis;
      }
      break;

    case PIVOTE:
      gererPivot();
      if (currentMillis - stateMillis >= DUREE_PIVOTE) {
        currentState = MARCHE;
      }
      break;
  }
}

int lireDistance(unsigned long currentMillis) {
  static int distance;
  static unsigned long previousMillis = 0;
  const unsigned long DUREE_LECTURE = 100;

  if (currentMillis - previousMillis >= DUREE_LECTURE) {
    previousMillis = currentMillis;
    distance = capteurDistance.distanceCm();

    Serial.print("Distance: ");
    Serial.println(distance);
  }
  return distance;
}

void gererMarche() {
  int vitessePwm = (VITESSE_CROISIERE_POURCENT / 100.0) * PWM_MAX;

  digitalWrite(M1_IN2, LOW);
  digitalWrite(M1_IN1, HIGH);
  analogWrite(M1_PWM, vitessePwm);

  digitalWrite(M2_IN2, LOW);
  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_PWM, vitessePwm);

  led.setColor(0, 0, 0, 0);
  for (int i = 0; i < NOMBRE_DE_LED_A_ACTIVER; i++) {
    led.setColorAt(ledsDevant[i], 0, 255, 0);
  }
  led.show();
}

void gererArret() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);

  led.setColor(255, 0, 0);
  led.show();
}

void gererLent() {
  int vitessePwm = (VITESSE_LENTE_POURCENT / 100.0) * PWM_MAX;

  digitalWrite(M1_IN2, LOW);
  digitalWrite(M1_IN1, HIGH);
  analogWrite(M1_PWM, vitessePwm);

  digitalWrite(M2_IN2, LOW);
  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_PWM, vitessePwm);

  led.setColor(0, 0, 0, 0);
  for (int i = 0; i < NOMBRE_DE_LED_A_ACTIVER; i++) {
    led.setColorAt(ledsArriere[i], 255, 255, 0);
  }
  led.show();
}

void gererRecule() {
  int vitessePwm = (VITESSE_LENTE_POURCENT / 100.0) * PWM_MAX;

  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_PWM, vitessePwm);

  digitalWrite(M2_IN2, HIGH);
  digitalWrite(M2_IN1, LOW);
  analogWrite(M2_PWM, vitessePwm);

  led.setColor(255, 0, 0);
  led.show();
}

void gererPivot() {
  int vitessePwm = (VITESSE_CROISIERE_POURCENT / 100.0) * PWM_MAX;

  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_PWM, vitessePwm);

  digitalWrite(M2_IN2, LOW);
  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_PWM, vitessePwm);

  led.setColor(255, 0, 0);
  led.show();
}
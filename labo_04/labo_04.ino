#include <MeAuriga.h>

#include <Adafruit_seesaw.h>

#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

MeGyro gyro(0, 0x69);

#define NB_IR 5

Adafruit_seesaw ss;
//int sensorValues[NB_IR];  // Tableau pour stocker les valeurs des capteurs

unsigned long currentTime = 0;
String commande = "";
bool is_calibrated = false;
bool cmdConnue = true;

const int VITESSE_PIVOT = 40;
const int VITESSE_LIGNE = 60;
const float LIMITE_INTEGRALE = 1000;
float integral = 0;
float lastError = 0;

struct Capteur {
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue;
  int valeurNormalisee;
};

Capteur capteurs[NB_IR];

//variables labos 3---------------------------------------------------------------------------------------------------------------------------------------
const float KP_LAB3 = 4.0;
const float KI_LAB3 = 1.0;
const float KD_LAB3 = 2.0;
const float LIMITE_INTEGRALE_LAB3 = 30.0;

const int VITESSE = 90;
const float PERIMETRE_ROUE = 6.5 * PI;
const float DISTANCE_X = 50.0;
const int DISTANCE_ARRET_OBSTACLE = 30;

const float ANGLE_PIVOT_1 = 85.0;
const float ANGLE_PIVOT_2 = 83.0;
const float ANGLE_PIVOT_3 = 172.0;

const unsigned long DELAI_STABILISATION = 300;
const unsigned long DUREE_FIN_A = 2000;

MeUltrasonicSensor capteurDistance(PORT_10);

enum Etat_LAB3 {
  ATTENTE_DEPART,
  SEG_A,
  PIVOT_1,
  SEG_B,
  PIVOT_2,
  SEG_C,
  PIVOT_3,
  FIN_A,
  TERMINE_LAB3
};
Etat_LAB3 etat_LAB3 = ATTENTE_DEPART;
bool premiereFoisLab3 = true;

void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void) {
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}

enum Etat {
  BOOT,
  ATTENTE,
  LAB3,
  CALIB,
  SUIVRE_LIGNE,
  CMD_INVALIDE,
  NON_CALIB
};

Etat etat = BOOT;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  gyro.begin();

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);
  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);

  Serial.begin(115200);
  led.setpin(LEDPIN);

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1)
      ;
  }
  Serial.println("Connexion réussie au LyneTracker!");
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();

  if (etat != SUIVRE_LIGNE) {
    Encoder_1.loop();
    Encoder_2.loop();
    gyro.update();
  }

  switch (etat) {
    case BOOT:
      gererBoot();
      break;
    case ATTENTE:
      gererAttente();
      break;
    case LAB3:
      gererLab3();
      break;
    case CALIB:
      gererCalib();
      break;
    case SUIVRE_LIGNE:
      gererSuivreLigne();
      break;
    case CMD_INVALIDE:
      gererCmdInvalide();
      break;
    case NON_CALIB:
      gererNonCalib();
      break;
  }
}

void gererBoot() {
  static bool premiereFois = true;
  const unsigned long DELAI_BOOT = 3000;
  static unsigned long tempsEntree = 0;

  if (premiereFois) {
    premiereFois = false;
    tempsEntree = currentTime;
    Serial.println("Etat : BOOT");
  }

  bool transition = (currentTime - tempsEntree >= DELAI_BOOT);

  if (transition) {
    etat = ATTENTE;
    premiereFois = true;
  }
}

void gererAttente() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : ATTENTE");
  }
  //transition
  if (commande == "lab3") {
    etat = LAB3;
    commande = "";
    premiereFois = true;
  } else if (commande == "cal") {
    etat = CALIB;
    commande = "";
    premiereFois = true;
  } else if (commande == "ligne" && is_calibrated) {
    etat = SUIVRE_LIGNE;
    commande = "";
    premiereFois = true;
  } else if (commande == "ligne" && !is_calibrated) {
    etat = NON_CALIB;
    commande = "";
    premiereFois = true;
  } else if (cmdConnue == false) {
    etat = CMD_INVALIDE;
    commande = "";
    premiereFois = true;
    cmdConnue = true;
  }
}

void gererLab3() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : LAB3");
  }

  switch (etat_LAB3) {
    case ATTENTE_DEPART:
      gererAttenteDepart();
      break;
    case SEG_A:
      gererSegA();
      break;
    case PIVOT_1:
      gererPivot1();
      break;
    case SEG_B:
      gererSegB();
      break;
    case PIVOT_2:
      gererPivot2();
      break;
    case SEG_C:
      gererSegC();
      break;
    case PIVOT_3:
      gererPivot3();
      break;
    case FIN_A:
      gererFinA();
      break;
    case TERMINE_LAB3:
      break;
  }

  bool termine = (etat_LAB3 == TERMINE_LAB3);
  bool arret = (commande == "stop");

  if (termine || arret) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    led.setColor(0, 0, 0, 0);
    led.show();
    etat_LAB3 = ATTENTE_DEPART;
    premiereFoisLab3 = true;
    commande = "";
    etat = ATTENTE;
    premiereFois = true;
  }
}

void gererCalib() {
  static bool premiereFois = true;
  static float anglePrecedent = 0;
  static float rotationTotale = 0;
  const float ROTATION_CIBLE = 360;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : CALIB");

    for (int i = 0; i < NB_IR; i++) {
      capteurs[i].valeurMin = 1023;
      capteurs[i].valeurMax = 0;
    }

    rotationTotale = 0;
    anglePrecedent = gyro.getAngleZ();

    Encoder_1.setTarPWM(-VITESSE_PIVOT);
    Encoder_2.setTarPWM(-VITESSE_PIVOT);
  }

  calibrerCapteurs();

  float angleActuel = gyro.getAngleZ();
  float difference = angleActuel - anglePrecedent;
  if (difference > 180) difference -= 360;
  if (difference < -180) difference += 360;
  rotationTotale += difference;
  anglePrecedent = angleActuel;

  bool termine = (abs(rotationTotale) >= ROTATION_CIBLE);
  bool arret = (commande == "stop");

  if (termine || arret) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    if (termine) is_calibrated = true;
    commande = "";
    etat = ATTENTE;
    premiereFois = true;
  }
}

void lireCapteurs() {
  for (int i = 0; i < NB_IR; i++) {
    int val = ss.analogRead(i);
    capteurs[i].valeurLue = val;
  }
}

void calibrerCapteurs() {
  lireCapteurs();
  for (int i = 0; i < NB_IR; i++) {
    if (capteurs[i].valeurLue < capteurs[i].valeurMin) capteurs[i].valeurMin = capteurs[i].valeurLue;
    if (capteurs[i].valeurLue > capteurs[i].valeurMax) capteurs[i].valeurMax = capteurs[i].valeurLue;
  }
}

double capteurLectureNormalisee(int index) {
  double valeur = ((capteurs[index].valeurLue - capteurs[index].valeurMin) * 1.0) / (capteurs[index].valeurMax - capteurs[index].valeurMin) * 1000.0;
  return constrain(valeur, 0, 1000);
}

void normaliserValeurs() {
  for (int i = 0; i < NB_IR; i++) {
    capteurs[i].valeurNormalisee = capteurLectureNormalisee(i);
  }
}

float calculerPositionLigne() {
  float numerateur = 0;
  float denominateur = 0;
  float position;

  for (int i = 0; i < NB_IR; i++) {
    float valeur = 1000 - capteurs[i].valeurNormalisee;
    numerateur += valeur * (i - 2);
    denominateur += valeur;
  }

  position = numerateur / denominateur * 1000;
  return position;
}

float computePID(float position, float consigne) {
  const float kp = 0.1;
  const float ki = 0.01;
  const float kd = 0.01;

  float error = position - consigne;

  integral += error;
  integral = constrain(integral, -LIMITE_INTEGRALE, LIMITE_INTEGRALE);

  float derivative = error - lastError;
  lastError = error;

  float output = kp * error + ki * integral + kd * derivative;

  return output;
}

void gererSuivreLigne() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : SUIVRE_LIGNE");
    integral = 0;
    lastError = 0;
  }

  lireCapteurs();
  normaliserValeurs();
  float position = calculerPositionLigne();

  if (isnan(position)) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
  } else {
    float correction = computePID(position, 0.0);

    int vitesseDroite = constrain(-VITESSE_LIGNE + correction, -255, 255);
    int vitesseGauche = constrain(VITESSE_LIGNE + correction, -255, 255);

    Encoder_1.setMotorPwm(vitesseDroite);
    Encoder_2.setMotorPwm(vitesseGauche);
  }

  if (commande == "stop") {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
    etat = ATTENTE;
    commande = "";
    premiereFois = true;
  }
}

void gererCmdInvalide() {
  static bool premiereFois = true;
  static unsigned long previousMillis = 0;
  static bool allume = false;
  static unsigned long tempsEntree = 0;
  const unsigned long DELAI_Etat = 3000;

  if (premiereFois) {
    premiereFois = false;
    tempsEntree = currentTime;
    Serial.println("Etat : CMD_INVALIDE");
  }

  if (currentTime - previousMillis >= 200) {
    previousMillis = currentTime;
    allume = !allume;

    if (allume) {
      led.setColor(0, 20, 0, 0);
    } else {
      led.setColor(0, 0, 0, 0);
    }
    led.show();
  }

  bool termine = (currentTime - tempsEntree >= DELAI_Etat);
  bool arret = (commande == "stop");

  if (termine || arret) {
    premiereFois = true;
    etat = ATTENTE;
    led.setColor(0, 0, 0, 0);
    led.show();
    allume = false;
  }
}

void gererNonCalib() {
  static bool premiereFois = true;
  static unsigned long previousMillis = 0;
  static bool allume = false;
  static unsigned long tempsEntree = 0;
  const unsigned long DELAI_Etat = 3000;
  int rate = 200;

  if (premiereFois) {
    premiereFois = false;
    tempsEntree = currentTime;
    Serial.println("Etat : NON_CALIB");
  }

  if (currentTime - previousMillis >= rate) {
    previousMillis = currentTime;
    allume = !allume;

    if (allume) {
      led.setColor(0, 20, 20, 0);
    } else {
      led.setColor(0, 0, 0, 0);
    }
    led.show();
  }

  bool termine = (currentTime - tempsEntree >= DELAI_Etat);
  bool arret = (commande == "stop");

  if (termine || arret) {
    premiereFois = true;
    etat = ATTENTE;
    led.setColor(0, 0, 0, 0);
    led.show();
    allume = false;
  }
}

void serialEvent() {
  if (!Serial.available()) return;

  commande = Serial.readStringUntil('\n');
  commande.trim();

  if (commande.length() >= 3 && (uint8_t)commande[0] == 0xFF && (uint8_t)commande[1] == 0x55) {
    commande.remove(0, 2);
    commande.remove(commande.length() - 1);
  }

  if (commande != "stop" && commande != "lab3" && commande != "cal" && commande != "ligne" && commande != "") {
    Serial.println("cmd INCONNUE");
    cmdConnue = false;
  } else {
    Serial.print("cmd reçu : ");
    Serial.println(commande);
    cmdConnue = true;
  }
}

//Fonctions labo03---------------------------------------------------------------------------------------------------------------------------------------------------------

float differenceAngle(float angleActuel, float angleReference) {
  float difference = angleActuel - angleReference;
  if (difference > 180) difference -= 360;
  if (difference < -180) difference += 360;
  return difference;
}

void afficherProgression(int nbreLed) {
  for (int i = 1; i <= 7; i++) {
    if (i <= nbreLed) {
      led.setColor(i, 0, 0, 20);
    } else {
      led.setColor(i, 0, 0, 0);
    }
  }
  led.show();
}

// Attente de 3 secondes avant le départ
void gererAttenteDepart() {
  static unsigned long tempsEntree = 0;
  const unsigned long DELAI_DEPART = 3000;

  //ENTRÉE
  if (premiereFoisLab3) {
    premiereFoisLab3 = false;
    tempsEntree = currentTime;
    Serial.println("Etat : ATTENTE_DEPART");
  }

  //TRANSITION
  bool transition = (currentTime - tempsEntree >= DELAI_DEPART);

  if (transition) {
    premiereFoisLab3 = true;
    etat_LAB3 = SEG_A;
  }
}

// Segment A : avance de DISTANCE_X cm en ligne droite (correction gyro PID), DEL 12
void gererSegA() {
  static long positionDepart = 0;
  static float angleDepart = 0;
  static float erreurPrecedente = 0;
  static float integrale = 0;
  static unsigned long tempsPrecedent = 0;

  //ENTRÉE
  if (premiereFoisLab3) {
    premiereFoisLab3 = false;

    positionDepart = Encoder_1.getCurPos();
    angleDepart = gyro.getAngleZ();
    erreurPrecedente = 0;
    integrale = 0;
    tempsPrecedent = currentTime;

    Encoder_1.setTarPWM(-VITESSE);
    Encoder_2.setTarPWM(VITESSE);

    Serial.println("Etat : SEG_A");
  }

  //ACTION : correction gyro (PID)
  long degresParcourus = abs(Encoder_1.getCurPos() - positionDepart);
  float distanceParcourue = (degresParcourus / 360.0) * PERIMETRE_ROUE;

  float erreur = differenceAngle(gyro.getAngleZ(), angleDepart);

  float dt = (currentTime - tempsPrecedent) / 1000.0;
  tempsPrecedent = currentTime;
  integrale += erreur * dt;
  integrale = constrain(integrale, -LIMITE_INTEGRALE_LAB3, LIMITE_INTEGRALE_LAB3);

  float correction = KP_LAB3 * erreur + KI_LAB3 * integrale + KD_LAB3 * (erreur - erreurPrecedente);
  erreurPrecedente = erreur;

  Encoder_1.setTarPWM(-VITESSE - correction);
  Encoder_2.setTarPWM(VITESSE - correction);

  int nbreLedAllume = map(distanceParcourue, 0, DISTANCE_X, 0, 7);
  nbreLedAllume = constrain(nbreLedAllume, 0, 7);

  led.setColor(12, 0, 20, 0);
  afficherProgression(nbreLedAllume);

  //TRANSITION
  bool transition = (distanceParcourue >= DISTANCE_X);

  if (transition) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);

    afficherProgression(0);

    premiereFoisLab3 = true;
    etat_LAB3 = PIVOT_1;
  }
}

// Pivot 1 : 90 degrés vers la gauche
void gererPivot1() {
  static float angleDepart = 0;
  static bool moteursArretes = false;
  static unsigned long tempsArret = 0;

  //ENTRÉE
  if (premiereFoisLab3) {
    premiereFoisLab3 = false;
    moteursArretes = false;
    angleDepart = gyro.getAngleZ();

    Encoder_1.setTarPWM(-VITESSE_PIVOT);
    Encoder_2.setTarPWM(-VITESSE_PIVOT);
    Serial.println("Etat : PIVOT_1");
  }

  //ACTION
  float difference = differenceAngle(gyro.getAngleZ(), angleDepart);

  if (!moteursArretes && abs(difference) >= ANGLE_PIVOT_1) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    moteursArretes = true;
    tempsArret = currentTime;
  }

  //TRANSITION
  bool transition = moteursArretes && (currentTime - tempsArret >= DELAI_STABILISATION);

  if (transition) {
    premiereFoisLab3 = true;
    etat_LAB3 = SEG_B;
  }
}

// Segment B : avance jusqu'à 30 cm de l'obstacle, DEL 11
void gererSegB() {
  static float angleDepart = 0;
  static float erreurPrecedente = 0;
  static int distanceInitiale = 0;
  static float integrale = 0;
  static unsigned long tempsPrecedent = 0;

  //ENTRÉE
  if (premiereFoisLab3) {
    premiereFoisLab3 = false;
    Serial.println("Etat : SEG_B");

    angleDepart = gyro.getAngleZ();
    erreurPrecedente = 0;
    integrale = 0;
    tempsPrecedent = currentTime;
    distanceInitiale = capteurDistance.distanceCm();

    Encoder_1.setTarPWM(-VITESSE);
    Encoder_2.setTarPWM(VITESSE);
  }

  //ACTION
  int distanceDetecte = capteurDistance.distanceCm();

  float erreur = differenceAngle(gyro.getAngleZ(), angleDepart);

  float dt = (currentTime - tempsPrecedent) / 1000.0;
  tempsPrecedent = currentTime;
  integrale += erreur * dt;
  integrale = constrain(integrale, -LIMITE_INTEGRALE_LAB3, LIMITE_INTEGRALE_LAB3);

  float correction = KP_LAB3 * erreur + KI_LAB3 * integrale + KD_LAB3 * (erreur - erreurPrecedente);
  erreurPrecedente = erreur;

  Encoder_1.setTarPWM(-VITESSE - correction);
  Encoder_2.setTarPWM(VITESSE - correction);

  // Barre de progression (protection contre la division par zéro)
  int nbreLedAllume = 0;
  if (distanceInitiale > DISTANCE_ARRET_OBSTACLE) {
    nbreLedAllume = map(distanceDetecte, distanceInitiale, DISTANCE_ARRET_OBSTACLE, 0, 7);
    nbreLedAllume = constrain(nbreLedAllume, 0, 7);
  }

  led.setColor(11, 0, 20, 0);
  afficherProgression(nbreLedAllume);

  //TRANSITION
  bool transition = (distanceDetecte > 0 && distanceDetecte <= DISTANCE_ARRET_OBSTACLE);

  if (transition) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);

    afficherProgression(0);

    premiereFoisLab3 = true;
    etat_LAB3 = PIVOT_2;
  }
}

// Pivot 2 : 90 degrés vers la droite
void gererPivot2() {
  static float angleDepart = 0;
  static bool moteursArretes = false;
  static unsigned long tempsArret = 0;

  //ENTRÉE
  if (premiereFoisLab3) {
    premiereFoisLab3 = false;
    moteursArretes = false;
    angleDepart = gyro.getAngleZ();

    Encoder_1.setTarPWM(VITESSE_PIVOT);
    Encoder_2.setTarPWM(VITESSE_PIVOT);
    Serial.println("Etat : PIVOT_2");
  }

  //ACTION
  float difference = differenceAngle(gyro.getAngleZ(), angleDepart);

  if (!moteursArretes && abs(difference) >= ANGLE_PIVOT_2) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    moteursArretes = true;
    tempsArret = currentTime;
  }

  //TRANSITION
  bool transition = moteursArretes && (currentTime - tempsArret >= DELAI_STABILISATION);

  if (transition) {
    premiereFoisLab3 = true;
    etat_LAB3 = SEG_C;
  }
}

// Segment C : avance de DISTANCE_X cm en ligne droite (correction gyro PID), DEL 10
void gererSegC() {
  static long positionDepart = 0;
  static float angleDepart = 0;
  static float erreurPrecedente = 0;
  static float integrale = 0;
  static unsigned long tempsPrecedent = 0;

  //ENTRÉE
  if (premiereFoisLab3) {
    premiereFoisLab3 = false;

    positionDepart = Encoder_1.getCurPos();
    angleDepart = gyro.getAngleZ();
    erreurPrecedente = 0;
    integrale = 0;
    tempsPrecedent = currentTime;

    Encoder_1.setTarPWM(-VITESSE);
    Encoder_2.setTarPWM(VITESSE);

    Serial.println("Etat : SEG_C");
  }

  //ACTION : correction gyro (PID)
  long degresParcourus = abs(Encoder_1.getCurPos() - positionDepart);
  float distanceParcourue = (degresParcourus / 360.0) * PERIMETRE_ROUE;

  float erreur = differenceAngle(gyro.getAngleZ(), angleDepart);

  float dt = (currentTime - tempsPrecedent) / 1000.0;
  tempsPrecedent = currentTime;
  integrale += erreur * dt;
  integrale = constrain(integrale, -LIMITE_INTEGRALE_LAB3, LIMITE_INTEGRALE_LAB3);

  float correction = KP_LAB3 * erreur + KI_LAB3 * integrale + KD_LAB3 * (erreur - erreurPrecedente);
  erreurPrecedente = erreur;

  Encoder_1.setTarPWM(-VITESSE - correction);
  Encoder_2.setTarPWM(VITESSE - correction);

  int nbreLedAllume = map(distanceParcourue, 0, DISTANCE_X, 0, 7);
  nbreLedAllume = constrain(nbreLedAllume, 0, 7);

  led.setColor(10, 0, 20, 0);
  afficherProgression(nbreLedAllume);

  //TRANSITION
  bool transition = (distanceParcourue >= DISTANCE_X);

  if (transition) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);

    afficherProgression(0);

    premiereFoisLab3 = true;
    etat_LAB3 = PIVOT_3;
  }
}

// Pivot 3 : 180 degrés
void gererPivot3() {
  static float angleEntre = 0;

  //ENTRÉE
  if (premiereFoisLab3) {
    premiereFoisLab3 = false;
    Serial.println("Etat : PIVOT_3");

    angleEntre = gyro.getAngleZ();

    Encoder_1.setTarPWM(VITESSE_PIVOT);
    Encoder_2.setTarPWM(VITESSE_PIVOT);
  }

  //ACTION
  float angleParcouru = abs(differenceAngle(gyro.getAngleZ(), angleEntre));

  //TRANSITION
  bool transition = (angleParcouru >= ANGLE_PIVOT_3);

  if (transition) {
    premiereFoisLab3 = true;
    etat_LAB3 = FIN_A;
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
  }
}

// Fin A : DEL 8 à 12 clignotent en vert à 10 Hz (état final)
void gererFinA() {
  static unsigned long tempsEntree = 0;
  static unsigned long previousMillis = 0;
  static bool allume = false;
  const unsigned long DEMI_PERIODE = 50;

  //ENTRÉE
  if (premiereFoisLab3) {
    premiereFoisLab3 = false;
    tempsEntree = currentTime;
    Serial.println("Etat : FIN_A");
  }

  //ACTION : clignotement
  if (currentTime - previousMillis >= DEMI_PERIODE) {
    previousMillis = currentTime;
    allume = !allume;

    for (int i = 8; i <= 12; i++) {
      if (allume) {
        led.setColor(i, 0, 20, 0);
      } else {
        led.setColor(i, 0, 0, 0);
      }
    }
    led.show();
  }

  //TRANSITION
  bool transition = (currentTime - tempsEntree >= DUREE_FIN_A);

  if (transition) {
    premiereFoisLab3 = true;
    etat_LAB3 = TERMINE_LAB3;
  }
}
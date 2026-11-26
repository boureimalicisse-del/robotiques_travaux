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

const int VITESSE_PIVOT_LIGNE = 60;
const int VITESSE_LIGNE = 60;
const float LIMITE_INTEGRALE = 1000;
const int SEUIL_LIGNE = 500;
const int SEUIL_BARRE = 500;
const float CONSIGNE_LIGNE = 0.0;
const unsigned long DELAI_FIN_LIGNE = 300;
const unsigned long DELAI_CONFIRMATION_BARRE = 40;
const float ANGLE_PIVOT_LIGNE = 162.0;
const unsigned long DELAI_STABILISATION = 300;
float integral = 0;
float lastError = 0;

struct Capteur {
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue;
  int valeurNormalisee;
};

Capteur capteurs[NB_IR];

MeUltrasonicSensor capteurDistance(PORT_10);

//CONSTANTE VITESSE
const int VITESSE_MIN = 50;
int vitesseCroisiere = VITESSE_LIGNE;

//buzzer
#define BUZZERPIN 45
MeBuzzer buzzer;
const int FREQUENCE_KLAXON = 1000;
const unsigned long DUREE_KLAXON = 200;

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
  ATTENTE,
  CALIB,
  DEMARRAGE,
  RECHERCHE_LIGNE,
  SUIVRE_LIGNE,
  VIRAGE,
  APPROCHE_COLIS,
  DEMI_TOUR,
  ATTENTIF,
  VERIFICATION,
  STATIONNEMENT,
  MANUEL,
  TERMINE
};

Etat etat = ATTENTE;

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
  buzzer.setpin(BUZZERPIN);

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
    case ATTENTE:
      gererAttente();
      break;
    case CALIB:
      gererCalib();
      break;
    case DEMARRAGE:
      gererDemarrage();
      break;
    case RECHERCHE_LIGNE:
      gererRechercheLigne();
      break;
    case SUIVRE_LIGNE:
      gererSuivreLigne();
      break;
    case VIRAGE:
      gererVirage();
      break;
    case APPROCHE_COLIS:
      gererApprocheColis();
      break;
    case DEMI_TOUR:
      gererDemiTour();
      break;
    case ATTENTIF:
      gererAttentif();
      break;
    case VERIFICATION:
      gererVerification();
      break;
    case STATIONNEMENT:
      gererStationnement();
      break;
    case MANUEL:
      gererManuel();
      break;
    case TERMINE:
      gererTermine();
      break;
  }
}

void gererAttente() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : ATTENTE");
  }
  //transition
  if (commande == "cal") {
    etat = CALIB;
    commande = "";
    premiereFois = true;
  } else if (commande == "ligne" && is_calibrated) {
    etat = SUIVRE_LIGNE;
    commande = "";
    premiereFois = true;
  } else if (commande == "ligne" && !is_calibrated) {
    Serial.println("Calibration requise : envoyer cal");
    commande = "";
  }
}

void gererCalib() {
  static bool premiereFois = true;
  static float anglePrecedent = 0;
  static float rotationTotale = 0;
  const float ROTATION_CIBLE = 360;
  const float COMPENSATION_INERTIE = 10;

  if (premiereFois) {
    premiereFois = false;
    is_calibrated = false;
    Serial.println("Etat : CALIB");

    for (int i = 0; i < NB_IR; i++) {
      capteurs[i].valeurMin = 1023;
      capteurs[i].valeurMax = 0;
    }

    rotationTotale = 0;
    anglePrecedent = gyro.getAngleZ();

    Encoder_1.setTarPWM(-VITESSE_PIVOT_LIGNE);
    Encoder_2.setTarPWM(-VITESSE_PIVOT_LIGNE);
  }

  calibrerCapteurs();

  float angleActuel = gyro.getAngleZ();
  float difference = angleActuel - anglePrecedent;
  if (difference > 180) difference -= 360;
  if (difference < -180) difference += 360;
  rotationTotale += difference;
  anglePrecedent = angleActuel;

  bool termine = (abs(rotationTotale) >= ROTATION_CIBLE - COMPENSATION_INERTIE);
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

//demi tour pour continuer a suivre la ligne
void gererDemiTour() {
  static bool premiereFois = true;
  static bool enPivot = false;
  static unsigned long tempsArret = 0;
  static float angleDepartPivot = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    enPivot = false;
    tempsArret = currentTime;
    Serial.println("Etat : DEMI_TOUR");
  }

  //ACTION
  float angleParcouru = 0;

  if (!enPivot) {
    if (currentTime - tempsArret >= DELAI_STABILISATION) {
      angleDepartPivot = gyro.getAngleZ();
      enPivot = true;
      Encoder_1.setTarPWM(-VITESSE_PIVOT_LIGNE);
      Encoder_2.setTarPWM(-VITESSE_PIVOT_LIGNE);
    }
  } else {
    angleParcouru = abs(differenceAngle(gyro.getAngleZ(), angleDepartPivot));
  }

  //TRANSITION
  bool termine = enPivot && (angleParcouru >= ANGLE_PIVOT_LIGNE);
  bool arret = (commande == "stop");

  if (termine || arret) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
    if (arret) {
      etat = ATTENTE;
      commande = "";
    } else {
      etat = SUIVRE_LIGNE;
    }
    premiereFois = true;
  }
}
//suivre la ligne
void gererSuivreLigne() {
  static bool premiereFois = true;
  static bool finDetectable = false;///empêche le robot de déclarer une fin de ligne tant qu’il n’a pas d’abord retrouvé la ligne
  static bool ligneDisparue = false;
  static bool barreEnCours = false;
  static unsigned long tempsDisparition = 0;
  static unsigned long tempsDebutBarre = 0;
  static float derniereCorrection = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : SUIVRE_LIGNE");
    integral = 0;
    lastError = 0;
    finDetectable = false;
    ligneDisparue = false;
    barreEnCours = false;
    derniereCorrection = 0;
  }

  //ACTION
  lireCapteurs();
  normaliserValeurs();

  int plusSombre = 1000;
  int plusClair = 0;
  for (int i = 0; i < NB_IR; i++) {
    if (capteurs[i].valeurNormalisee < plusSombre) plusSombre = capteurs[i].valeurNormalisee;
    if (capteurs[i].valeurNormalisee > plusClair) plusClair = capteurs[i].valeurNormalisee;
  }

  bool ligneVisible = (plusSombre < SEUIL_LIGNE);
  bool barreDetectee = (plusClair < SEUIL_BARRE);

  if (!barreDetectee) {
    barreEnCours = false;
  } else if (!barreEnCours) {
    barreEnCours = true;
    tempsDebutBarre = currentTime;
  }
  bool barreConfirmee = barreEnCours && (currentTime - tempsDebutBarre >= DELAI_CONFIRMATION_BARRE);

  if (ligneVisible) {
    if (!barreDetectee) finDetectable = true;
    ligneDisparue = false;
    derniereCorrection = computePID(calculerPositionLigne(), CONSIGNE_LIGNE);
  } else if (!ligneDisparue) {
    ligneDisparue = true;
    tempsDisparition = currentTime;
  }

  int vitesseDroite = constrain(-VITESSE_LIGNE + derniereCorrection, -255, 255);
  int vitesseGauche = constrain(VITESSE_LIGNE + derniereCorrection, -255, 255);

  Encoder_1.setMotorPwm(vitesseDroite);
  Encoder_2.setMotorPwm(vitesseGauche);

  //TRANSITION
  bool finDeLigne = finDetectable && (barreConfirmee || (ligneDisparue && (currentTime - tempsDisparition >= DELAI_FIN_LIGNE)));
  bool arret = (commande == "stop");

  if (arret) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
    etat = ATTENTE;
    commande = "";
    premiereFois = true;
  } else if (finDeLigne) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    if (barreConfirmee) Serial.println("Fin de ligne : barre");
    else Serial.println("Fin de ligne : perte");
    etat = DEMI_TOUR;
    premiereFois = true;
  }
}

//Recevoir les commades bluetooth
void serialEvent() {
  if (!Serial.available()) return;

  String ligne = Serial.readStringUntil('\n');
  ligne.trim();

  if (ligne.length() >= 3 && (uint8_t)ligne[0] == 0xFF && (uint8_t)ligne[1] == 0x55) {
    ligne.remove(0, 2);
    ligne.remove(ligne.length() - 1);
  }

  traiterCommande(ligne);
}

//Fonctions labo03

float differenceAngle(float angleActuel, float angleReference) {
  float difference = angleActuel - angleReference;
  if (difference > 180) difference -= 360;
  if (difference < -180) difference += 360;
  return difference;
}
//fonction pour gestion d'etat du projet
void gererDemarrage() {
}

void gererRechercheLigne() {
}

void gererVirage() {
}

void gererApprocheColis() {
}

void gererAttentif() {
}

void gererVerification() {
}

void gererStationnement() {
}

void gererManuel() {
}

void gererTermine() {
}

//commandes
void commandeDeplacement(char direction) {
}

void commandeKlaxon() {
  buzzer.tone(FREQUENCE_KLAXON, DUREE_KLAXON);
}

void commandeLumiereAnneau(int r, int g, int b) {
  led.setColor(0, constrain(r, 0, 255), constrain(g, 0, 255), constrain(b, 0, 255));
  led.show();
}

void commandeLumiereDel(int i, int r, int g, int b) {
  if (i < 1 || i > LEDNUM) return;
  led.setColor(i, constrain(r, 0, 255), constrain(g, 0, 255), constrain(b, 0, 255));
  led.show();
}

void commandeVitesse(int pwm) {
  vitesseCroisiere = constrain(pwm, VITESSE_MIN, 255);
  Serial.print("Vitesse : ");
  Serial.println(vitesseCroisiere);
}

void commandeCalibration() {
  commande = "cal";
}

void commandeGo() {
  commande = "ligne";
}

void commandeStop() {
  commande = "stop";
}

void commandeManuel() {
}

int compterChamps(const String& ligne) {
  int nb = 1;
  for (unsigned int i = 0; i < ligne.length(); i++) {
    if (ligne[i] == ',') nb++;
  }
  return nb;
}

String extraireChamp(const String& ligne, int index) { //index : le numero du champs voulu
  int debut = 0;
  for (int i = 0; i < index; i++) {
    debut = ligne.indexOf(',', debut) + 1;
    if (debut == 0) return "";
  }
  int fin = ligne.indexOf(',', debut);
  if (fin == -1) fin = ligne.length();
  return ligne.substring(debut, fin);
}

void traiterCommande(String ligne) {
  if (ligne == "") return;

  String nom = extraireChamp(ligne, 0);
  int nbChamps = compterChamps(ligne);
  bool reconnue = true;

  if (nbChamps == 1 && (nom == "W" || nom == "A" || nom == "S" || nom == "D" || nom == "T")) {
    commandeDeplacement(nom[0]);
  } else if (nbChamps == 1 && nom == "E") {
    commandeKlaxon();
  } else if (nbChamps == 4 && nom == "l") {
    commandeLumiereAnneau(extraireChamp(ligne, 1).toInt(), extraireChamp(ligne, 2).toInt(), extraireChamp(ligne, 3).toInt());
  } else if (nbChamps == 5 && nom == "l") {
    commandeLumiereDel(extraireChamp(ligne, 1).toInt(), extraireChamp(ligne, 2).toInt(), extraireChamp(ligne, 3).toInt(), extraireChamp(ligne, 4).toInt());
  } else if (nbChamps == 2 && nom == "p") {
    commandeVitesse(extraireChamp(ligne, 1).toInt());
  } else if (nbChamps == 1 && nom == "C") {
    commandeCalibration();
  } else if (nbChamps == 1 && nom == "GO") {
    commandeGo();
  } else if (nbChamps == 1 && nom == "STOP") {
    commandeStop();
  } else if (nbChamps == 1 && nom == "MAN") {
    commandeManuel();
  } else if (nbChamps == 1 && (nom == "cal" || nom == "ligne" || nom == "stop")) {
    commande = nom;
  } else {
    reconnue = false;
  }

  if (reconnue) {
    Serial.print("j'ai exécuté ");
    Serial.println(ligne);
  } else {
    Serial.println("cmd INCONNUE");
  }
}
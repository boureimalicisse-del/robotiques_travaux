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

struct Capteur {
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue;
  int valeurNormalisee;
};

Capteur capteurs[NB_IR];

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

  Encoder_1.loop();
  Encoder_2.loop();

  gyro.update();

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

void gererLab3() {}

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
    for (int i = 0; i < NB_IR; i++) {
      //Serial.println(capteurs[i].valeurMin);
      //Serial.println(capteurs[i].valeurMax);
    }
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    if (termine) is_calibrated = true;
    commande = "";
    etat = ATTENTE;
    premiereFois = true;
  }
}

void gererSuivreLigne() {
  static bool premiereFois = true;
  static unsigned long dernierAffichage = 0;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : SUIVRE_LIGNE");
  }

  lireCapteurs();
  normaliserValeurs();
  float position = calculerPositionLigne();

  if (currentTime - dernierAffichage >= 200) {
    dernierAffichage = currentTime;
    Serial.println(position);
  }

  if (commande == "stop") {
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

  bool transition = (currentTime - tempsEntree >= DELAI_Etat);
  if (transition) {
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

  bool transition = (currentTime - tempsEntree >= DELAI_Etat);
  if (transition) {
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

  if (commande != "stop" && commande != "lab3" && commande != "cal" && commande != "ligne" && commande != "") {
    Serial.println("cmd INCONNUE");
    cmdConnue = false;
  } else {
    Serial.print("cmd reçu : ");
    Serial.println(commande);
    cmdConnue = true;
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

void lireCapteurs() {
  for (int i = 0; i < NB_IR; i++) {
    int val = ss.analogRead(i);
    capteurs[i].valeurLue = val;
  }
}

float calculerPositionLigne(){
  float numerateur = 0;
  float denominateur = 0;
  float position;

  for(int i=0;i<NB_IR;i++){
    float valeur = 1000 - capteurs[i].valeurNormalisee;
    numerateur += valeur * (i - 2);
    denominateur += valeur;
  }

  position = numerateur / denominateur * 1000;
  return position;
}



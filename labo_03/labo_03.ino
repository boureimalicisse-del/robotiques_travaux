/*
 * Boureima Ali Cissé
 * DA : 2414842  ->  deux derniers chiffres : 42
 * DA[6] = 2 (pair)  ->  premier pivot vers la gauche
 * Variations :
 *   - Pivot 1 : 90 degrés vers la GAUCHE
 *   - Pivot 2 : 90 degrés vers la DROITE
 *   - Pivot 3 : 180 degrés
 */

#include <MeAuriga.h>

#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor capteurDistance(PORT_10);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

MeGyro gyro(0, 0x69);

const float KP = 3.0;
const float KI = 4.0;
const float KD = 1.0;
const float LIMITE_INTEGRALE = 30.0;
const float INTEGRALE_DEPART = -5.0;  // compense roue gauche tordue

const int VITESSE = 90;
const int VITESSE_PIVOT = 40;  // 30 : manque de force
const float PERIMETRE_ROUE = 6.5 * PI;
const float DISTANCE_X = 120.0;
const int DISTANCE_ARRET_OBSTACLE = 30;

const float ANGLE_PIVOT_1 = 85.0;   // 90 degrés visés : dépassait
const float ANGLE_PIVOT_2 = 83.0;   // 90 degrés visés : dépassait
const float ANGLE_PIVOT_3 = 172.0;  // 180 degrés visés: dépassait
const float DEPASSEMENT_GAUCHE = 90.0 - ANGLE_PIVOT_1;

const unsigned long DELAI_STABILISATION = 300;

float distanceSegB = 0;
float angleRetour = 0;
float distanceRetour = 0;

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
  ATTENTE_DEPART,
  SEG_A,
  PIVOT_1,
  SEG_B,
  PIVOT_2,
  SEG_C,
  PIVOT_3,
  FIN_A,
  PIVOT_RETOUR,
  SEG_RETOUR,
  PIVOT_FINAL,
  FIN_B
};

Etat etat = ATTENTE_DEPART;

unsigned long currentTime = 0;

void setup() {
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
}

void loop() {
  currentTime = millis();

  Encoder_1.loop();
  Encoder_2.loop();

  gyro.update();

  switch (etat) {
    case ATTENTE_DEPART: gererAttenteDepart(); break;
    case SEG_A: gererSegA(); break;
    case PIVOT_1: gererPivot1(); break;
    case SEG_B: gererSegB(); break;
    case PIVOT_2: gererPivot2(); break;
    case SEG_C: gererSegC(); break;
    case PIVOT_3: gererPivot3(); break;
    case FIN_A: gererFinA(); break;
    case PIVOT_RETOUR: gererPivotRetour(); break;
    case SEG_RETOUR: gererSegRetour(); break;
    case PIVOT_FINAL: gererPivotFinal(); break;
    case FIN_B: gererFinB(); break;
  }
}

// Affiche la barre de progression sur les DEL 1 à 7 (bleu)
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

// Arc_en_ciel
void couleurArcEnCiel(byte pos, byte &r, byte &g, byte &b) {
  if (pos < 85) {
    r = 255 - pos * 3;
    g = pos * 3;
    b = 0;
  } else if (pos < 170) {
    pos -= 85;
    r = 0;
    g = 255 - pos * 3;
    b = pos * 3;
  } else {
    pos -= 170;
    r = pos * 3;
    g = 0;
    b = 255 - pos * 3;
  }
}

void animerArcEnCiel() {
  static unsigned long derniereImage = 0;
  static byte decalage = 0;
  const unsigned long DELAI_IMAGE = 30;

  if (currentTime - derniereImage < DELAI_IMAGE) return;
  derniereImage = currentTime;

  for (int i = 1; i <= LEDNUM; i++) {
    byte r, g, b;
    couleurArcEnCiel((byte)(decalage + i * 256 / LEDNUM), r, g, b);
    led.setColor(i, r / 10, g / 10, b / 10);
  }
  led.show();

  decalage += 3;
}

// Attente de 3 secondes avant le départ
void gererAttenteDepart() {
  static bool premiereFois = true;
  static unsigned long tempsEntree = 0;
  const unsigned long DELAI_DEPART = 3000;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    tempsEntree = currentTime;
    Serial.println("Etat : ATTENTE_DEPART");
  }

  //TRANSITION
  bool transition = (currentTime - tempsEntree >= DELAI_DEPART);

  if (transition) {
    premiereFois = true;
    etat = SEG_A;
  }
}

// Segment A : avance de DISTANCE_X cm en ligne droite (correction gyro PID), DEL 12
void gererSegA() {
  static bool premiereFois = true;
  static long positionDepart = 0;
  static float angleDepart = 0;
  static float erreurPrecedente = 0;
  static float integrale = 0;
  static unsigned long tempsPrecedent = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;

    positionDepart = Encoder_1.getCurPos();
    angleDepart = gyro.getAngleZ();
    erreurPrecedente = 0;
    integrale = INTEGRALE_DEPART;
    tempsPrecedent = currentTime;

    Encoder_1.setTarPWM(-VITESSE);
    Encoder_2.setTarPWM(VITESSE);

    Serial.println("Etat : SEG_A");
  }

  //ACTION : correction gyro (PID)
  long degresParcourus = abs(Encoder_1.getCurPos() - positionDepart);
  float distanceParcourue = (degresParcourus / 360.0) * PERIMETRE_ROUE;

  float erreur = gyro.getAngleZ() - angleDepart;

  float dt = (currentTime - tempsPrecedent) / 1000.0;
  tempsPrecedent = currentTime;
  integrale += erreur * dt;
  integrale = constrain(integrale, -LIMITE_INTEGRALE, LIMITE_INTEGRALE);

  float correction = KP * erreur + KI * integrale + KD * (erreur - erreurPrecedente);
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

    premiereFois = true;
    etat = PIVOT_1;
  }
}

// Pivot 1 : 90 degrés vers la gauche
void gererPivot1() {
  static bool premiereFois = true;
  static float angleDepart = 0;
  static bool moteursArretes = false;
  static unsigned long tempsArret = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    moteursArretes = false;
    angleDepart = gyro.getAngleZ();

    Encoder_1.setTarPWM(-VITESSE_PIVOT);
    Encoder_2.setTarPWM(-VITESSE_PIVOT);
    Serial.println("Etat : PIVOT_1");
  }

  //ACTION
  float difference = gyro.getAngleZ() - angleDepart;

  if (!moteursArretes && abs(difference) >= ANGLE_PIVOT_1) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    moteursArretes = true;
    tempsArret = currentTime;
  }

  //TRANSITION
  bool transition = moteursArretes && (currentTime - tempsArret >= DELAI_STABILISATION);

  if (transition) {
    premiereFois = true;
    etat = SEG_B;
  }
}

// Segment B : avance jusqu'à 30 cm de l'obstacle, DEL 11
void gererSegB() {
  static bool premiereFois = true;
  static float angleDepart = 0;
  static float erreurPrecedente = 0;
  static int distanceInitiale = 0;
  static long positionDepart = 0;
  static float integrale = 0;
  static unsigned long tempsPrecedent = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : SEG_B");

    angleDepart = gyro.getAngleZ();
    erreurPrecedente = 0;
    integrale = INTEGRALE_DEPART;
    tempsPrecedent = currentTime;
    distanceInitiale = capteurDistance.distanceCm();
    positionDepart = Encoder_1.getCurPos();

    Encoder_1.setTarPWM(-VITESSE);
    Encoder_2.setTarPWM(VITESSE);
  }

  //ACTION
  int distanceDetecte = capteurDistance.distanceCm();

  float erreur = gyro.getAngleZ() - angleDepart;

  float dt = (currentTime - tempsPrecedent) / 1000.0;
  tempsPrecedent = currentTime;
  integrale += erreur * dt;
  integrale = constrain(integrale, -LIMITE_INTEGRALE, LIMITE_INTEGRALE);

  float correction = KP * erreur + KI * integrale + KD * (erreur - erreurPrecedente);
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

    // Distance parcourue pendant SEG_B (pour le retour)
    long degresParcourus = abs(Encoder_1.getCurPos() - positionDepart);
    distanceSegB = (degresParcourus / 360.0) * PERIMETRE_ROUE;
    Serial.print("distanceSegB : ");
    Serial.println(distanceSegB);

    afficherProgression(0);

    premiereFois = true;
    etat = PIVOT_2;
  }
}

// Pivot 2 : 90 degrés vers la droite
void gererPivot2() {
  static bool premiereFois = true;
  static float angleDepart = 0;
  static bool moteursArretes = false;
  static unsigned long tempsArret = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    moteursArretes = false;
    angleDepart = gyro.getAngleZ();

    Encoder_1.setTarPWM(VITESSE_PIVOT);
    Encoder_2.setTarPWM(VITESSE_PIVOT);
    Serial.println("Etat : PIVOT_2");
  }

  //ACTION
  float difference = gyro.getAngleZ() - angleDepart;

  if (!moteursArretes && abs(difference) >= ANGLE_PIVOT_2) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    moteursArretes = true;
    tempsArret = currentTime;
  }

  //TRANSITION
  bool transition = moteursArretes && (currentTime - tempsArret >= DELAI_STABILISATION);

  if (transition) {
    premiereFois = true;
    etat = SEG_C;
  }
}

// Segment C : avance de DISTANCE_X cm en ligne droite (correction gyro PID), DEL 10
void gererSegC() {
  static bool premiereFois = true;
  static long positionDepart = 0;
  static float angleDepart = 0;
  static float erreurPrecedente = 0;
  static float integrale = 0;
  static unsigned long tempsPrecedent = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;

    positionDepart = Encoder_1.getCurPos();
    angleDepart = gyro.getAngleZ();
    erreurPrecedente = 0;
    integrale = INTEGRALE_DEPART;
    tempsPrecedent = currentTime;

    Encoder_1.setTarPWM(-VITESSE);
    Encoder_2.setTarPWM(VITESSE);

    Serial.println("Etat : SEG_C");
  }

  //ACTION : correction gyro (PID)
  long degresParcourus = abs(Encoder_1.getCurPos() - positionDepart);
  float distanceParcourue = (degresParcourus / 360.0) * PERIMETRE_ROUE;

  float erreur = gyro.getAngleZ() - angleDepart;

  float dt = (currentTime - tempsPrecedent) / 1000.0;
  tempsPrecedent = currentTime;
  integrale += erreur * dt;
  integrale = constrain(integrale, -LIMITE_INTEGRALE, LIMITE_INTEGRALE);

  float correction = KP * erreur + KI * integrale + KD * (erreur - erreurPrecedente);
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

    premiereFois = true;
    etat = PIVOT_3;
  }
}

// Pivot 3 : 180 degrés
void gererPivot3() {
  static bool premiereFois = true;
  static float angleEntre = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : PIVOT_3");

    angleEntre = gyro.getAngleZ();

    Encoder_1.setTarPWM(VITESSE_PIVOT);
    Encoder_2.setTarPWM(VITESSE_PIVOT);
  }

  //ACTION
  float angleParcouru = abs(gyro.getAngleZ() - angleEntre);

  //TRANSITION
  bool transition = (angleParcouru >= ANGLE_PIVOT_3);

  if (transition) {
    premiereFois = true;
    etat = FIN_A;
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
  }
}

// Fin A : DEL 8 à 12 clignotent en vert à 10 Hz
void gererFinA() {
  static bool premiereFois = true;
  static unsigned long tempsEntree = 0;
  static unsigned long previousMillis = 0;
  static bool allume = false;
  const unsigned long DEMI_PERIODE = 50;
  const unsigned long DUREE_FIN_A = 2000;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
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
    premiereFois = true;
    etat = PIVOT_RETOUR;
  }
}

// Retour : pivote vers la case départ
void gererPivotRetour() {
  static bool premiereFois = true;
  static float angleDepart = 0;
  static float angleArret = 0;
  static bool moteursArretes = false;
  static unsigned long tempsArret = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    moteursArretes = false;

    float distanceAvant = 2 * DISTANCE_X;
    angleRetour = atan(distanceSegB / distanceAvant) * 180.0 / PI;
    distanceRetour = sqrt(distanceAvant * distanceAvant + distanceSegB * distanceSegB);
    angleArret = max(angleRetour - DEPASSEMENT_GAUCHE, 0.0);

    Serial.print("angleRetour : ");
    Serial.print(angleRetour);
    Serial.print("   distanceRetour : ");
    Serial.println(distanceRetour);

    angleDepart = gyro.getAngleZ();

    Encoder_1.setTarPWM(-VITESSE_PIVOT);
    Encoder_2.setTarPWM(-VITESSE_PIVOT);

    Serial.println("Etat : PIVOT_RETOUR");
  }

  //ACTION
  animerArcEnCiel();

  float difference = gyro.getAngleZ() - angleDepart;

  if (!moteursArretes && abs(difference) >= angleArret) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    moteursArretes = true;
    tempsArret = currentTime;
  }

  //TRANSITION
  bool transition = moteursArretes && (currentTime - tempsArret >= DELAI_STABILISATION);

  if (transition) {
    premiereFois = true;
    etat = SEG_RETOUR;
  }
}

// Retour : avance en diagonale jusqu'à la case départ
void gererSegRetour() {
  static bool premiereFois = true;
  static long positionDepart = 0;
  static float angleDepart = 0;
  static float erreurPrecedente = 0;
  static float integrale = 0;
  static unsigned long tempsPrecedent = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;

    positionDepart = Encoder_1.getCurPos();
    angleDepart = gyro.getAngleZ();
    erreurPrecedente = 0;
    integrale = INTEGRALE_DEPART;
    tempsPrecedent = currentTime;

    Encoder_1.setTarPWM(-VITESSE);
    Encoder_2.setTarPWM(VITESSE);

    Serial.println("Etat : SEG_RETOUR");
  }

  //ACTION : correction gyro (PID)
  animerArcEnCiel();

  long degresParcourus = abs(Encoder_1.getCurPos() - positionDepart);
  float distanceParcourue = (degresParcourus / 360.0) * PERIMETRE_ROUE;

  float erreur = gyro.getAngleZ() - angleDepart;

  float dt = (currentTime - tempsPrecedent) / 1000.0;
  tempsPrecedent = currentTime;
  integrale += erreur * dt;
  integrale = constrain(integrale, -LIMITE_INTEGRALE, LIMITE_INTEGRALE);

  float correction = KP * erreur + KI * integrale + KD * (erreur - erreurPrecedente);
  erreurPrecedente = erreur;

  Encoder_1.setTarPWM(-VITESSE - correction);
  Encoder_2.setTarPWM(VITESSE - correction);

  //TRANSITION
  bool transition = (distanceParcourue >= distanceRetour);

  if (transition) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);

    premiereFois = true;
    etat = PIVOT_FINAL;
  }
}

// Pivot final : se remet dans le sens du départ
void gererPivotFinal() {
  static bool premiereFois = true;
  static float angleDepart = 0;
  static float angleFinal = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;

    angleFinal = 180.0 - angleRetour - DEPASSEMENT_GAUCHE;
    angleDepart = gyro.getAngleZ();

    Encoder_1.setTarPWM(-VITESSE_PIVOT);
    Encoder_2.setTarPWM(-VITESSE_PIVOT);

    Serial.println("Etat : PIVOT_FINAL");
  }

  //ACTION
  animerArcEnCiel();

  float difference = gyro.getAngleZ() - angleDepart;

  //TRANSITION
  bool transition = abs(difference) >= angleFinal;

  if (transition) {
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);

    premiereFois = true;
    etat = FIN_B;
  }
}

// Fin B : arrivé à la case départ, anneau couleur fixe
void gererFinB() {
  static bool premiereFois = true;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;

    led.setColor(0, 20, 0);
    led.show();

    Serial.println("Etat : FIN_B");
  }
}
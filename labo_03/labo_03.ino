/*
 * Boureima Ali Cissé
 * DA : 2414842  ->  DA[6] = 2 (pair)
 * Variations :
 *   - Pivot 1 : 90° vers la GAUCHE
 *   - Pivot 2 : 90° vers la DROITE (sens opposé)
 *   - Pivot 3 : 180°
 */

#include <MeAuriga.h>

#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor capteurDistance(PORT_10);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

const float KP = 3.0;
const float KD = 1.0;

const int VITESSE = 100;
const int VITESSE_PIVOT = 35;  // vitesse pour angle environ 90
const float PERIMETRE_ROUE = 6.5 * PI;
const float DISTANCE_X = 50.0;

MeGyro gyro(0, 0x69);

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
  FIN_A
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
  }
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


void gererAttenteDepart() {
  static bool premiereFois = true;
  static unsigned long tempsEntree = 0;
  const unsigned long DELAI_DEPART = 3000;

  if (premiereFois) {
    premiereFois = false;
    tempsEntree = currentTime;
    Serial.println("Etat : ATTENTE_DEPART");
  }

  bool transition = (currentTime - tempsEntree >= DELAI_DEPART);

  if (transition) {
    premiereFois = true;
    etat = SEG_A;
  }
}

// Segment A : avancement avec gyroscope
void gererSegA() {
  static bool premiereFois = true;
  static long positionDepart = 0;
  static float angleDepart = 0;
  static float erreurPrecedente = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;

    positionDepart = Encoder_1.getCurPos();
    angleDepart = gyro.getAngleZ();
    erreurPrecedente = 0;

    Encoder_1.setTarPWM(-VITESSE);
    Encoder_2.setTarPWM(VITESSE);

    Serial.println("Etat : SEG_A");
  }

  //  ACTION : calcul de la correction gyro (PD)
  long degresParcourus = abs(Encoder_1.getCurPos() - positionDepart);
  float distanceParcourue = (degresParcourus / 360.0) * PERIMETRE_ROUE;
  int nbreLedAllume = 0;

  float erreur = gyro.getAngleZ() - angleDepart;
  float correction = KP * erreur + KD * (erreur - erreurPrecedente);
  erreurPrecedente = erreur;

  Serial.println(correction);
  Encoder_1.setTarPWM(-VITESSE - correction);
  Encoder_2.setTarPWM(VITESSE - correction);

  nbreLedAllume = map(distanceParcourue, 0, DISTANCE_X, 0, 7);
  nbreLedAllume = constrain(nbreLedAllume, 0, 7);

  afficherProgression(nbreLedAllume);

  led.setColor(12, 0, 20, 0);
  led.show();


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

// Pivot 1 : 90 vers la gauche
void gererPivot1() {
  static bool premiereFois = true;
  static float angleDepart = 0;

  //ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    angleDepart = gyro.getAngleZ();

    Encoder_1.setTarPWM(-VITESSE_PIVOT);
    Encoder_2.setTarPWM(-VITESSE_PIVOT);
    Serial.println("Etat : PIVOT_1");
  }

  //ACTION
  float difference = gyro.getAngleZ() - angleDepart;

  //Serial.print("depart: ");
  //Serial.print(angleDepart);
  //Serial.print("  actuel: ");
  //Serial.print(gyro.getAngleZ());
  //Serial.print("  diff: ");
  //Serial.println(difference);

  //TRANSITION
  bool transition = abs(difference) >= 90;

  if (transition) {
    premiereFois = true;
    etat = SEG_B;

    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
  }
}

void gererSegB() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : SEG_B");
  }

  //ACTION

  bool transition = false;  // À FAIRE

  if (transition) {
    premiereFois = true;
    etat = PIVOT_2;
  }
}

void gererPivot2() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : PIVOT_2");
  }

  // ACTION

  bool transition = false;

  if (transition) {
    premiereFois = true;
    etat = SEG_C;
  }
}

void gererSegC() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : SEG_C");
  }

  // ACTION

  bool transition = false;

  if (transition) {
    premiereFois = true;
    etat = PIVOT_3;
  }
}

void gererPivot3() {
  static bool premiereFois = true;

  // ENTRÉE
  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : PIVOT_3");
  }

  // ACTION

  // TRANSITION
  bool transition = false;  // À FAIRE

  if (transition) {
    premiereFois = true;
    etat = FIN_A;
  }
}

void gererFinA() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : FIN_A");
  }

  // ACTION
  // À FAIRE : clignotement

  // Pas de transition , état final pour l'instant
}
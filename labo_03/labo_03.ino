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
  Serial.begin(115200);
  led.setpin(LEDPIN);
}

void loop() {
  currentTime = millis();

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


void gererAttenteDepart() {
  static bool premiereFois = true;
  static unsigned long tempsEntree = 0;
  const unsigned long DELAI_DEPART = 3000;

  if (premiereFois) {
    premiereFois = false;
    tempsEntree = currentTime;
    Serial.println("Etat : ATTENTE_DEPART");
  }

  //action

  bool transition = (currentTime - tempsEntree >= DELAI_DEPART);

  if (transition) {
    premiereFois = true;
    etat = SEG_A;
  }
}

void gererSegA() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : SEG_A");
  }

  //ACTION

  bool transition = false;  //A changer

  if (transition) {
    premiereFois = true;
    etat = PIVOT_1;
  }
}

void gererPivot1() {
  static bool premiereFois = true;

  if (premiereFois) {
    premiereFois = false;
    Serial.println("Etat : PIVOT_1");
  }

  //ACTION

  bool transition = false;  // A changer

  if (transition) {
    premiereFois = true;
    etat = SEG_B;
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
  bool transition = false;   // À FAIRE

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
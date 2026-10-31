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
int sensorValues[NB_IR];  // Tableau pour stocker les valeurs des capteurs

unsigned long currentTime = 0;

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
  bool transition = false;  //a faire

  if (transition) {
    premiereFois = true;
  }
}

void gererLab3() {}

void gererCalib() {}

void gererSuivreLigne() {}

void gererCmdInvalide() {}

void gererNonCalib() {}

#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>

// Définir les broches
const int startButtonPin = 2; // Broche pour le bouton de départ
const int greenLEDPin = 3; // Broche pour la LED verte
const int redLEDPin = 4; // Broche pour la LED rouge
const int finishSensorPin = 5; // Broche pour le capteur de fin (tactile ou infrarouge)
const int dfPlayerRxPin = 16; // Broche RX pour le DFPlayer Mini
const int dfPlayerTxPin = 17; // Broche TX pour le DFPlayer Mini

// Définir les variables de temps et de distance
const float trackLength = 12.0; // Longueur de la piste en mètres
const unsigned long timeoutDuration = 20000; // Timeout de fin de course en millisecondes (20 secondes)
const unsigned long countdownDuration = 3000; // Durée du décompte en millisecondes (3 secondes)

// Variables pour gérer l'état de la course
unsigned long startTime = 0;
unsigned long finishTime = 0;
bool raceInProgress = false;
bool countdownInProgress = false;

// Initialiser le DFPlayer Mini
DFRobotDFPlayerMini dfPlayer;

// Déclarations de fonctions (prototypes)
void startRace();
void finishRace();
void resetRace();
void displayResults(float time, float speed);

void setup() {
  // Configurer les broches
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(finishSensorPin, INPUT);

  // Initialiser les LEDs
  digitalWrite(greenLEDPin, HIGH); // LED verte allumée (prêt)
  digitalWrite(redLEDPin, LOW); // LED rouge éteinte

  // Initialiser la communication série pour le DFPlayer Mini
  Serial.begin(9600);
  dfPlayer.begin(Serial);

  // Configurer le DFPlayer Mini
  dfPlayer.volume(10); // Régler le volume
}

void startRace() {
  countdownInProgress = true;
  dfPlayer.play(1); // Jouer la musique de décompte
  delay(countdownDuration); // Attendre la fin du décompte
  countdownInProgress = false;
  raceInProgress = true;
  startTime = millis();
  digitalWrite(greenLEDPin, LOW); // Éteindre la LED verte
  digitalWrite(redLEDPin, HIGH); // Allumer la LED rouge
}

void finishRace() {
  finishTime = millis();
  raceInProgress = false;
  unsigned long raceDuration = finishTime - startTime;
  float raceTimeSeconds = raceDuration / 1000.0;
  float speed = (trackLength / raceTimeSeconds) * 3.6; // Convertir m/s en km/h
  displayResults(raceTimeSeconds, speed);
  resetRace();
}

void resetRace() {
  digitalWrite(greenLEDPin, HIGH); // Allumer la LED verte
  digitalWrite(redLEDPin, LOW); // Éteindre la LED rouge
  startTime = 0;
  finishTime = 0;
}

void displayResults(float time, float speed) {
  // Afficher les résultats sur l'afficheur (à implémenter selon votre matériel)
  Serial.print("Temps: ");
  Serial.print(time);
  Serial.print(" s, Vitesse: ");
  Serial.print(speed);
  Serial.println(" km/h");
}

void loop() {
  if (digitalRead(startButtonPin) == LOW && !raceInProgress && !countdownInProgress) {
    startRace();
  }

  if (raceInProgress) {
    if (digitalRead(finishSensorPin) == HIGH) {
      finishRace();
    } else if (millis() - startTime > timeoutDuration) {
      resetRace();
    }
  }
}

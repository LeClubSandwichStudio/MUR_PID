/**
 * Exemple de code pour le circuit intégré CD4051B.
 */
#include <TimerOne.h>

// Broches d'adresse
//const byte PIN_ENABLE = 2;
const byte PIN_ADDR_A = 0;
const byte PIN_ADDR_B = 1;
const byte PIN_ADDR_C = 2;
 
// Broche de signal
const byte PIN_SIG = A11;
 
/** Fonction setup() */
void setup(){
  
  // Place les broches d'adresse en sortie et à LOW
  pinMode(PIN_ADDR_A, OUTPUT);
  pinMode(PIN_ADDR_B, OUTPUT); 
  pinMode(PIN_ADDR_C, OUTPUT); 
  digitalWrite(PIN_ADDR_A, LOW);
  digitalWrite(PIN_ADDR_B, LOW);
  digitalWrite(PIN_ADDR_C, LOW);
  
  // Active le CD4051B
  //pinMode(PIN_ENABLE, OUTPUT);
  //digitalWrite(PIN_ENABLE, LOW);
 
  // Message de bienvenue
  Serial.begin(115200);
  Serial.println("Demonstration CD4051B");
}
 
/** Fonction loop() */
void loop(){
 
  // Pour chaque voie on renvoie la valeur sur le port série
  for(byte i = 0; i < 8; i++){
    Serial.print("Voie ");
    Serial.print(i,BIN);
    Serial.print(" : ");
    Serial.println(readAnalogMux(i));
  }
  
  // Delai pour l'affichage
  delay(100);
}
 
/** Fonction de lecture pour le CD4051B */
int readAnalogMux(byte channel) {
  
  // On sélectionne la voie
  digitalWrite(PIN_ADDR_A, bitRead(channel, 0));
  digitalWrite(PIN_ADDR_B, bitRead(channel, 1));
  digitalWrite(PIN_ADDR_C, bitRead(channel, 2));
  delay(1);
  // On lit la valeur courante
  return analogRead(PIN_SIG);
}

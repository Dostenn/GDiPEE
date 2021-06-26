#include <EmonLib.h>

#include "TimerOne.h" // Mise en place librairie Timer Interrupt pour lire les valeurs
#include "EmonLib.h"  // Mise en place librairie EmonLib
#include <math.h>

#define AnemometrePin (2) // Pin de l'anemometer 
#define GirouettePin (A4) // Pin de la girouette
#define GirouetteOffset 0; // définit le décalage de l'anémomètre par rapport au nord (magnétique)

int GirouetteValue; // analog value (Girouette)
int Cardinalite; // Passage Analog Value en Degree
int CalCardinalite; // Passage Degree en cardinalite
int LastValue; // Dernière valeur Cardinale obtenu

char Card[3];

float duree = 3; //Durée en secondes avant envoie des données

volatile bool IsSampleRequired; // Met valeur en vrai tout les "duree" ecoulé
volatile unsigned int TimerCount; // Permet le comptage
volatile unsigned long Rotations; // Comptage de tour utilise dans routine d'interruption
volatile unsigned long ContactBounceTime; // Minuterie pour éviter le saut de contact dans l'ISR

float VitesseVentMPH; // Vitesse en Miles par heure
float VitesseVentKmh; // Vitesse en Km/h

EnergyMonitor emon1; //Création Instance (Courant AC)

//___________
#define GIROUETTE  A0
#define pinILS 4
#define pi 3.1415
#define RayonDesBras 0.07

float nombreTourSec = 0;
int nombreTourMin = 0;
float vitesseVent;
float FEtalonage = 1;
int delaiSeconde1 = 1; // premier delai de 1 sec
int delaiSeconde2 = 5; // deuxieme delai de 5 sec
float gir(0);         //Direction girouette sur un instant
int nbGir = 0;        //nb d'occurence de mesure Anemo
float xi = 0;         //Valeur x et y qui seront attribué à
float yi = 0;         //l'angle de la girouette, basé sur le cercle trigonometrique

volatile unsigned int comptageILS = 0; // une variable utilisée dans une interruption doit être déclarée "volatile"

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long delaiAnemometre = 1000L;    //1 secondes
unsigned long delaiProgramme =  5000L;   //5 sec

void interruptILS() { //comptage de l'ILS
  comptageILS++;
}



//_________

void setup() {
  lcd.begin(16, 2); //LCD Taille (colonne, ligne)
  lcd.setRGB(colorR, colorG, colorB); //LCD Couleur (R,G,B)
  float tmp = duree * 200000;

  LastValue = 0;

  IsSampleRequired = false;

  TimerCount = 0;
  Rotations = 0; // Mettre Rotations à 0 prêt pour calcul

  Serial.begin(9600);

  pinMode(AnemometrePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(AnemometrePin), isr_rotation, FALLING);

  Serial.println("Speed (Km/h)\tCardinalite\tIrms\t\tPuissance\t|\tSpeed (Km/h)");

  // timer interupt
  Timer1.initialize(tmp);// Timer interrupt à chaque "duree"  qui passe
  Timer1.attachInterrupt(isr_timer);

  pinMode(buttonPin, INPUT);

  pinMode(pinILS, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinILS), interruptILS, RISING);
}


//Fonction qui converti en angle la valeur analogique
//RETURN : angle en degré
float getGirouetteAngle(int value) {
  float angle = 0;
  if (value >= 280 && value <= 292) angle = 270;          // x S   180
  if (value >= 628 && value <= 636) angle = 225;          // x SO  225
  if (value >= 940 && value <= 950) angle = 180;          // x O   270
  if (value >= 884 && value <= 892) angle = 135;          // x NO  315
  if (value >= 784 && value <= 792) angle = 90;            // x N   0
  if (value >= 458 && value <= 468) angle = 45;           // x NE  45
  if (value >= 88 && value <= 98)   angle = 0;           // x E   90
  if (value >= 180 && value <= 190) angle = 315;          // x SE  135
  return angle;
}


void loop() {

  //____________

  unsigned long currentMillis = millis(); // Temps reel passé
  //Toutes les 5 sec, compilation des valeurs et envoi au serveur
  if (currentMillis - previousMillis2 > 1000 * delaiSeconde2) {
    previousMillis2 = millis();

    //Nombre de tour(s)
    //Nombre d'impulsion par tour = 2
    nombreTourSec = (float)comptageILS / 2; //comptage du nombre de tours par seconde

    //2 * PI * R * nbTour --> donne la vitesse en m/s
    //conversion m/s en k/h -->multiplier par 3,6
    vitesseVent = (2 * pi * RayonDesBras * nombreTourSec * FEtalonage * 3.6) / delaiSeconde2; // formule pour le calcul de la vitesse du vent
    comptageILS = 0; // réinitialisation du comptage

  }
  //____________


  double Irms = emon1.calcIrms(1480);  // Calcul Irms

  getVentCardinalite();

  // Ne mettez à jour l'affichage que si le changement est supérieur à 5 degrés
  if (abs(CalCardinalite - LastValue) > 5) {
    LastValue = CalCardinalite;
  }

  if (IsSampleRequired) {
    // convertir Analogique en mp/h avec formule V=P(2.25/T)
    // V = P(2.25/2.5) = P * 0.9
    VitesseVentMPH = Rotations * 0.9;

    // convertir mp/h en km/h avec formule
    // V(mp/h) = V(km/h)*1.609
    VitesseVentKmh = VitesseVentMPH * 1.609;

    Rotations = 0; // Réinitialiser le compte pour le prochain échantillon

    IsSampleRequired = false;

    Serial.print(VitesseVentKmh); Serial.print("\t\t");
    Serial.print(CalCardinalite);
    getHeading(CalCardinalite); Serial.print("\t\t");

    emon1.current(8, 57);             // Current: input pin, calibration.
    Serial.print(Irms); Serial.print("\t\t");           // Irms
    Serial.print(Irms * 230.0); Serial.print("\t\t\t");         // Apparent power

    //affiche vitesse du vent
    Serial.println(vitesseVent);

  }
}

// gestionnaire d'isr pour l'interruption du minuteur
void isr_timer() {

  TimerCount++;

  if (TimerCount == 6)
  {
    IsSampleRequired = true;
    TimerCount = 0;
  }
}

// interruption qui incrémenter le nombre de rotations
void isr_rotation() {

  if ((millis() - ContactBounceTime) > 15 ) { // anti-saut de contact de commutation
    Rotations++;
    ContactBounceTime = millis();
  }
}



// Degree de la girouette
void getVentCardinalite() {

  GirouetteValue = analogRead(GirouettePin);
  Cardinalite = map(GirouetteValue, 0, 1023, 0, 359);
  CalCardinalite = Cardinalite + GirouetteOffset;

  if (CalCardinalite > 360)
    CalCardinalite = CalCardinalite - 360;

  if (CalCardinalite < 0)
    CalCardinalite = CalCardinalite + 360;

}

// Cardinalite de la girouette
void getHeading(int Cardinalite) {

  if (Cardinalite < 22) {
    Serial.print(" N");
    Card[0] = 'N';
    Card[1] = ' ';
  }
  else if (Cardinalite < 67) {
    Serial.print(" NW");
    Card[0] = 'N';
    Card[1] = 'W';
  }
  else if (Cardinalite < 112) {
    Serial.print(" W");
    Card[0] = 'W';
    Card[1] = ' ';
  }
  else if (Cardinalite < 157) {
    Serial.print(" SW");
    Card[0] = 'S';
    Card[1] = 'W';
  }
  else if (Cardinalite < 212) {
    Serial.print(" S");
    Card[0] = 'S';
    Card[1] = ' ';
  }
  else if (Cardinalite < 247) {
    Serial.print(" SE");
    Card[0] = 'S';
    Card[1] = 'E';
  }
  else if (Cardinalite < 292) {
    Serial.print(" E");
    Card[0] = 'E';
    Card[1] = ' ';
  }
  else if (Cardinalite < 337) {
    Serial.print(" NE");
    Card[0] = 'N';
    Card[1] = 'E';
  }
  else {
    Serial.print(" N");
    Card[0] = 'N';
    Card[1] = ' ';
  }
}

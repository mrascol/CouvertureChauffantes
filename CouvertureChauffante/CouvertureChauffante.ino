/*
 The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 http://www.arduino.cc/en/Tutorial/LiquidCrystalDisplay

*/


#include <LiquidCrystal.h>// include the library for LCD 
#include <Wire.h>     // include the library for EEPROM adressing
 
#define eeprom 0x50    //Address of 24LC128 eeprom chip
#define eepromSize 128  //Taille de l'EEPROM - ici 128kbits = 16Ko

// Debug MODE
bool dbgMode = 1;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize Bouton
int BtnPin = A6;

// Conf du contrast
int screenContrastPin=6;
const byte screenContrastLst[5]={200, 150, 100, 50, 10};
String screenContrastLib[5]={"=", "====", "========", "============", "================"};
byte screenContrastVal = 2;


// Conf du Delay en secondes 
const int autoCutLst[5]={-1, 10, 3600, 7200, 14400};
String autoCutLib[5]={"OFF", "30min", "1h", "2h", "4h"};
byte autoCutVal = 3;


//conf des libelles
const String couvList[4]={"FL", "FR", "RL", "RR"};

//Conf des températures
int consigne[2]={50,50};
int correctionTemp[4]={0,0,0,0};
int temperature[4]={0,0,0,0};

//Conf de l'expertMode
byte expertModeNbStep=3;
int expertModeConsigneFront[3]={10, 20, 30};
int expertModeConsigneRear[3]={15, 25, 35};
int expertModeStepLength[3]={5,10,15};  //Si la valeur vaut 61, on affichera "OFF"

//Initialisation des capteurs de temp
int sensorFL=A1;
int sensorFR=A0;
int sensorRL=A3;
int sensorRR=A2;
int B=3975;  // Alors ca je ne sais pas d'ou ca sort :-)

//Initialisation des fils resistifs
int chauffeFL=9;
int chauffeFR=10;
int chauffeRL=7;
int chauffeRR=8;

//Config de l'adresse EEPROM
unsigned int consigneEepromAddress[2] = {0,1};
unsigned int autoCutValEepromAddress = 4;
unsigned int screenContrastValEepromAddress = 5;
unsigned int correctionTempEepromAddress[4] = {6,7,8,9};
unsigned int expertModeNbStepEepromAddress = 10;
unsigned int expertModeConsigneFrontEepromAddress[3]={11, 12, 13};
unsigned int expertModeConsigneRearEepromAddress[3]={14, 15, 16};
unsigned int expertModeStepLengthEepromAddress[3]={17,18,19}; 

// Création du caractère Flèche
byte arrow[8] = {
  B00000,
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100
};

byte arrow_small[8] = {
  B00000,
  B00000,
  B00000,
  B00100,
  B01110,
  B10101,
  B00100
};

// Création du caractère Flèche
byte infini[8] = {
  B00000,
  B00000,
  B01110,
  B10001,
  B01110,
  B00000,
  B00000,
  B00000
};

void setup() {
  String fctName="setup";
  
  // Init Serial
  if (dbgMode >=1 ){
    Serial.begin(9600);
    while(!Serial);
  }
  
  // Btn initialize
  pinMode(BtnPin, INPUT);

  // Port pour le réglage contrast écran
  pinMode(screenContrastPin, OUTPUT);

  //capteur température init
  pinMode(sensorFL, INPUT);
  pinMode(sensorFR, INPUT);
  pinMode(sensorRL, INPUT);
  pinMode(sensorRR, INPUT);

  //Fil resistif init
  pinMode(chauffeFL, OUTPUT);
  digitalWrite(chauffeFL, LOW);
  pinMode(chauffeFR, OUTPUT);
  digitalWrite(chauffeFR, LOW);
  pinMode(chauffeRL, OUTPUT);
  digitalWrite(chauffeRL, LOW);
  pinMode(chauffeRR, OUTPUT);
  digitalWrite(chauffeRR, LOW);

  // Init de l'EEPROM
  Wire.begin();
  delay(500);

  
  // On va aller Lire le contenu de l'EEPROM
  // pour les consignes de températures
  // pour la valeur d'auto cut-off
  // pour le contrast
  // Les corrections de températures

  consigne[0] = readEEPROM(eeprom, consigneEepromAddress[0]);
  consigne[1] = readEEPROM(eeprom, consigneEepromAddress[1]);

  autoCutVal = readEEPROM(eeprom, autoCutValEepromAddress);
  screenContrastVal = readEEPROM(eeprom, screenContrastValEepromAddress);       
  correctionTemp[0] = readEEPROM(eeprom, correctionTempEepromAddress[0]);
  correctionTemp[1] = readEEPROM(eeprom, correctionTempEepromAddress[1]);
  correctionTemp[2] = readEEPROM(eeprom, correctionTempEepromAddress[2]);
  correctionTemp[3] = readEEPROM(eeprom, correctionTempEepromAddress[3]);   

  expertModeNbStep= readEEPROM(eeprom, expertModeNbStepEepromAddress);

  expertModeConsigneFront[0]= readEEPROM(eeprom, expertModeConsigneFrontEepromAddress[0]);
  expertModeConsigneFront[1]= readEEPROM(eeprom, expertModeConsigneFrontEepromAddress[1]);
  expertModeConsigneFront[2]= readEEPROM(eeprom, expertModeConsigneFrontEepromAddress[2]);

  expertModeConsigneRear[0]= readEEPROM(eeprom, expertModeConsigneRearEepromAddress[0]);
  expertModeConsigneRear[1]= readEEPROM(eeprom, expertModeConsigneRearEepromAddress[1]);
  expertModeConsigneRear[2]= readEEPROM(eeprom, expertModeConsigneRearEepromAddress[2]);

  expertModeStepLength[0]= readEEPROM(eeprom, expertModeStepLengthEepromAddress[0]);
  expertModeStepLength[1]= readEEPROM(eeprom, expertModeStepLengthEepromAddress[1]);
  expertModeStepLength[2]= readEEPROM(eeprom, expertModeStepLengthEepromAddress[2]);

  //Mise jour du contrast
  analogWrite(screenContrastPin, screenContrastLst[screenContrastVal]);
  // set up the LCD's number of columns and rows and contrast Init
  lcd.begin(16, 2);
  lcd.print(F("www.ae-rc.com"));
  lcd.setCursor(0,1);
  lcd.print(__DATE__);
  delay (3000);

  // Initialistion du caractère créé
  lcd.createChar(0, arrow);
  lcd.createChar(1, infini);
  lcd.createChar(2, arrow_small);
  
}

void loop() {
  String fctName="mainMenu";

  int posMenu=0;
  int posMenuNew=0;
  
  //Menu Construction
  String mainMenuLib[3];
  mainMenuLib[0]= "1.Quick Warming";
  mainMenuLib[1]= "2.Expert Mode";
  mainMenuLib[2]= "3.Setup";

  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(mainMenuLib[posMenu]);

  while ((1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 3);
  
    if (dbgMode>=1){Serial.println(fctName+F("|posMenuNew=")+String(posMenuNew));}
    
    // Si la position dans le menu a changé, alors on change l'affichage
    //Ici la touche Back ne sert à rien
    if ((posMenuNew == -2)){
      posMenuNew=posMenu;
    }

    //Si la touche Valid est pressee, on descend dans le bon menu
    if (posMenuNew == -1){
      posMenuNew=posMenu;
      switch (posMenu){
        case 0 :
          warmingMenu();
          break;
        case 1 :
          expertModeMenu();
          break;
        case 2 :
          setupMenu();
          break;
      }
    }

    posMenu=posMenuNew;
     
    lcd.clear();
    lcd.noCursor();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print(mainMenuLib[posMenu]);     
  }
}

// Fonction qui permet de faire la chauffe 
// IN : N/A
// OUT : N/A
void warmingMenu(){
  String fctName="warmingMenu";

  int posMenu=0;
  int posMenuNew=0;
  bool keepWarming=1;

  //On prends l'heure de démarrage
  unsigned long startWarmingTime=0;
  startWarmingTime=millis();
  

  //On initialise l'affichage
  lcd.clear();
  lcd.noCursor();
  lcd.noBlink();
  lcd.setCursor(0,0);
  lcd.print(F("    FL=__ FR=__"));
  lcd.setCursor(0,1);
  lcd.print(F("000 RL=__ RR=__"));
    
  while ((keepWarming==1)){
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture
    //1. read les 4 temps
    //2. ajuster la tension
    //3. Ajuster l'affichage
    temperature[0]=warmingCheckAdjust(sensorFL, chauffeFL, consigne[0], correctionTemp[0], 0, 7);
    temperature[1]=warmingCheckAdjust(sensorFR, chauffeFR, consigne[0], correctionTemp[1], 0, 13);
    temperature[2]=warmingCheckAdjust(sensorRL, chauffeRL, consigne[1], correctionTemp[2], 1, 7);
    temperature[3]=warmingCheckAdjust(sensorRR, chauffeRR, consigne[1], correctionTemp[3], 1, 13);
    
    //On met les températures à jour sur l'affichage pour ne pas perdre les "fleches" positionnées par warmingCheckAdjust
    lcd.setCursor(7,0);
    lcd.print(String(round(temperature[0])));
    lcd.setCursor(13,0);
    lcd.print(String(round(temperature[1])));
    lcd.setCursor(7,1);
    lcd.print(String(round(temperature[2])));
    lcd.setCursor(13,1);
    lcd.print(String(round(temperature[3])));

    //Mise à jour timer
    lcd.setCursor(0,1);
    lcd.print(String(padding(round((millis()-startWarmingTime)/1000/60),3)));
    

    posMenu=posMenuNew;
    
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 2);
  
    if (dbgMode>=1){Serial.println(fctName+F("|posMenuNew=")+String(posMenuNew));}
    
    // Si la touche UP ou DOWN est pressee, alors on passe dans l'écran de réglage des temps
    // Mais avant on coupe la chauffe
    if (posMenuNew != posMenu && posMenuNew>0){
      digitalWrite(chauffeFL, LOW);
      digitalWrite(chauffeFR, LOW);
      digitalWrite(chauffeRL, LOW);
      digitalWrite(chauffeRR, LOW);
      warmingSetup();

      //Puis on remet l'affichage à jour
      lcd.clear();
      lcd.noCursor();
      lcd.noBlink();
      lcd.setCursor(0,0);
      lcd.print("    FL=" + String((round(temperature[0]))) + F(" FR=") + String((round(temperature[1]))));
      lcd.setCursor(0,1);
      lcd.print(String(padding(round((millis()-startWarmingTime)/1000/60),3))+ F(" RL=") + String((round(temperature[2]))) + F(" RR=") + String((round(temperature[3]))));
      
      posMenuNew=posMenu;
    }

    // Si la touche Valid a été pressée -> on ne fait rien
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (posMenuNew ==-2){
      keepWarming=0;
      posMenuNew=posMenu;
    }

    // On Check si on a pas atteint la fin du delay
    if (dbgMode>=2){Serial.println(fctName+F("|startWarmingTime=")+String(startWarmingTime)+F("|millis=")+String(millis()));}
    if ((millis()-startWarmingTime)/1000 > autoCutLst[autoCutVal]){
      keepWarming=0;
    }
  }
  //On coupe la chauffe avant de sortir
  digitalWrite(chauffeFL, LOW);
  digitalWrite(chauffeFR, LOW);
  digitalWrite(chauffeRL, LOW);
  digitalWrite(chauffeRR, LOW);
}


// Fonction qui permet de faire la chauffe 
// IN : Le Step dans lequel on est (en mode Expert) 
// OUT : la valeur de la touche presse (-2 = Back / -1 = Valid)
int warmingMenuExpert(byte currentStep){
  String fctName="warmingMenuExpert";

  int posMenu=0;
  int posMenuNew=0;
  bool keepWarming=1;
  
  //Menu Construction
  String menuLib[2];
  unsigned long startWarmingTime;
  unsigned long remainingTime;

  startWarmingTime=millis();
  //On initialise l'affichage
  menuLib[0]= "Stp"+ String(currentStep+1) + " FL=__ R=__";
  menuLib[1]= "___m FL=__ R=__";
  posMenu=posMenuNew;
  lcd.clear();
  lcd.noCursor();
  lcd.noBlink();
  
  while ((keepWarming==1)){
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture
    //1. read les 4 temps
    //2. ajuster la tension
    //3. Ajuster l'affichage
    lcd.setCursor(0,0);
    lcd.print(menuLib[0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[1]);
    if (expertModeStepLength[currentStep]> 60){
      //affichage logo infini
      lcd.setCursor(0,1);
      lcd.print(F(" "));
      lcd.setCursor(1,1);
      lcd.write(byte(1));
      lcd.setCursor(2,1);
      lcd.write(byte(1));
    }

    
    temperature[0]=warmingCheckAdjust(sensorFL, chauffeFL, expertModeConsigneFront[currentStep], correctionTemp[0], 0, 17);
    temperature[1]=warmingCheckAdjust(sensorFR, chauffeFR, expertModeConsigneFront[currentStep], correctionTemp[1], 0, 17);
    temperature[2]=warmingCheckAdjust(sensorRL, chauffeRL, expertModeConsigneRear[currentStep], correctionTemp[2], 0, 17);
    temperature[3]=warmingCheckAdjust(sensorRR, chauffeRR, expertModeConsigneRear[currentStep], correctionTemp[3], 0, 17);
    
    //On met à jour l'affichage
    //Duree du Step - (heure courante -heure de début)
    remainingTime=expertModeConsigneFront[currentStep]-(round(millis()-startWarmingTime)/1000/60);    
    menuLib[0]= "Stp"+ String(currentStep+1) + " FL="+String((round(temperature[0])))+" FR="+String((round(temperature[1])));
    menuLib[1]= String(padding(remainingTime,3))+"m" + " FL="+ String((round(temperature[2])))+" RR="+String((round(temperature[3])));

    // On attend qu'un bouton soit pressé
    posMenu=0;
    posMenuNew = readBtn(posMenu, 0, 2);
    
    // si la touche back est pressee, alors on revient à l'écran d'avant
    // si la touche valid est pressee, alors on passe au step suivant
    // Dans tous les cas on retourne cette valeure
    if (posMenuNew < 0 ){
      keepWarming=0;
    }
  }
  //On coupe la chauffe avant de sortir
  digitalWrite(chauffeFL, LOW);
  digitalWrite(chauffeFR, LOW);
  digitalWrite(chauffeRL, LOW);
  digitalWrite(chauffeRR, LOW);
  return posMenuNew;

  
}

// Fonction qui permet de régler les consignes de température
// IN : le port du Sensor, la consigne pour ce port, la ligne pour l'affichage, la colonne pour l'affichage
// OUT : la nouvelle température mesurée
int warmingCheckAdjust(int sensorCurrent, int sensorChauffe, int consigneCurrent, int tempCorrectionCurrent, int ligneCurrent, int colonneCurrent){
  String fctName="warmingCheckAdjust";

  int readValue = 0;
  float resistance;
  float transformedValue;
  
  // Lecture de la température et conversion en °C
  readValue = analogRead(sensorCurrent);
  resistance=(float)(1023-readValue)*10000/readValue; 
  transformedValue=1/(log(resistance/10000)/B+1/298.15)-273.15;
  if (dbgMode>=1){Serial.print(fctName+F("|position=")+String(ligneCurrent)+F("/")+String(colonneCurrent)+F(" |mesured=")+String(transformedValue)+F(" |consigne=")+String(consigneCurrent)+F(" |correction=")+String(tempCorrectionCurrent));}

  //On check si on doit couper la chauffe
  //3 mode différents :
  //  - si je suis au dessus : je coupe
  //  - si je suis largement en dessous : j'allume
  //  - Si je suis à 2° près en dessous : j'allume pour 0.5s
  if (transformedValue+tempCorrectionCurrent>=consigneCurrent) {
      digitalWrite(sensorChauffe, LOW); 
      lcd.setCursor(colonneCurrent+2,ligneCurrent);
      lcd.print(F(" ")); 
      if (dbgMode>=1){Serial.println(fctName + F("| --> OFF"));}
  }
  else{
      if (transformedValue+tempCorrectionCurrent<consigneCurrent-2){
          digitalWrite(sensorChauffe, HIGH);
          lcd.setCursor(colonneCurrent+2,ligneCurrent);
          lcd.write(byte(0));  //Affichage de la flèche
          if (dbgMode>=1){Serial.println(fctName + F("| --> ON"));}
      }
      else
      {
          digitalWrite(sensorChauffe, HIGH);
          lcd.setCursor(colonneCurrent+2,ligneCurrent);
          lcd.write(byte(0));  //Affichage de la flèche
          if (dbgMode>=1){Serial.println(fctName + F("| --> ON_short"));}
          delay(500);
          digitalWrite(sensorChauffe, LOW);   
          lcd.setCursor(colonneCurrent+2,ligneCurrent);
          lcd.write(byte(2));    
      }
  }
  return transformedValue+tempCorrectionCurrent;
}

// Fonction qui permet de régler les consignes de température
// IN : N/A
// OUT : N/A
void warmingSetup(){
  String fctName="warmingSetupMenu";

  int posMenuNew=0;
  
  byte cursorPos[2]={4,14};
  byte cursorPosCurrent=0;
  
  bool keepSetuping=1;

  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Temp Setup:"));
  lcd.setCursor(0,1);
  lcd.print("FT=" + String(consigne[0]) + F("     RR=") + String(consigne[1]));

  //On positionne curseur on bon endroit et on le fait clignoter
  lcd.setCursor(cursorPos[cursorPosCurrent],1);
  lcd.cursor();
  lcd.blink();

  while ((keepSetuping==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(consigne[cursorPosCurrent], 20, 75);
  
    if (dbgMode>=1){Serial.println(fctName+F("|posMenuNew=")+String(posMenuNew));}
    
    // Si la touche Valide est pressee, alors on passe au réglage suivant
    if (posMenuNew == -1){
      if (cursorPosCurrent==1){
        cursorPosCurrent=0;
      }
      else{
        cursorPosCurrent=1;
      }
      posMenuNew=consigne[cursorPosCurrent];
    }
    
    // si la touche back est pressee, alors on revient à l'écran d'avant
    // Mais on sauvegarde qd meme la valeure
    if (posMenuNew==-2){
      posMenuNew=consigne[cursorPosCurrent];
      keepSetuping=0;
    }

    // On met à jour la consigne
    if (cursorPosCurrent==0){
     consigne[0]= posMenuNew;
     consigne[1]= posMenuNew; 
    }
    else {
     consigne[1]= posMenuNew;     
    }
    
    // On met à jour le texte et on affiche
    lcd.setCursor(0,1);
    lcd.print(F("                "));
    lcd.setCursor(0,1);
    lcd.print("FT=" + String(consigne[0]) + F("     RR=") + String(consigne[1]));
 
    //On positionne curseur on bon endroit et on le fait clignoter
    lcd.setCursor(cursorPos[cursorPosCurrent],1);
    lcd.cursor();
    lcd.blink();   
  }

  //Avant de sortir on enregistre les consignes dans l'EEPROM
  writeEEPROM(eeprom, consigneEepromAddress[0], consigne[0]);
  writeEEPROM(eeprom, consigneEepromAddress[1], consigne[1]);
  
  lcd.noCursor();
  lcd.noBlink();
}


// Fonction du Menu de paramétrage
// IN : N/A
// OUT : N/A
void setupMenu(){
  String fctName="setupMenu";

  int posMenu=0;
  int posMenuNew=0;
  bool keepMenu=1;
  //Menu Construction
  String menuLib[5][2];
  menuLib[0][0]= {"1.Contrast"};
  menuLib[0][1]= {String(screenContrastLib[screenContrastVal])};
  menuLib[1][0]= {"2.Cut-off Delay"};
  menuLib[1][1]= {String(autoCutLib[autoCutVal])};
  menuLib[2][0]= {"3.Calibrate"};
  menuLib[2][1]= {""};  
  menuLib[3][0]= {"4.Factory Reset"};
  menuLib[3][1]= {""};
  menuLib[4][0]= {"5.Version"};
  menuLib[4][1]= {"soft=1.1 HW=0.1"};
 

  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  while ((keepMenu==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 5);
  
    if (dbgMode>=1){Serial.println(fctName+"|posMenuNew="+String(posMenuNew));}
    
    // Si la touche Valide est pressee, alors on passe dans le menu suivant
    if (posMenuNew == -1){
      switch (posMenu){
        case 0 : // Config du contrast
          contrastConfig();
          menuLib[0][1]= {String(screenContrastLib[screenContrastVal])};
          break;
        case 1 : // auto cut-off delay
          autoCutConfig();
          menuLib[1][1]= {String(autoCutLib[autoCutVal])};
          break;
        case 2 : // Correction Temps
          correctionTempConfig();
          break;
        case 3 : //Restore Default
          restoreDefault();
          break;
      }

      posMenuNew=posMenu;
    }
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (posMenuNew ==-2){
      keepMenu=0;;
      posMenuNew=posMenu;
    }
    
    posMenu=posMenuNew;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(menuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[posMenu][1]);
     
  }
}


// Fonction du Menu du mode Expert
// IN : N/A
// OUT : N/A
void expertModeMenu(){
  String fctName="expertModeMenu";

  int posMenu=0;
  int posMenuNew=0;
  int returnedValue;
  bool keepStepping=1;
  byte i=0;
  bool keepMenu=1;
  //Menu Construction
  String menuLib[2][2];
  menuLib[0][0]= {"1.Expert Warming"};
  menuLib[0][1]= {""};
  menuLib[1][0]= {"2.Expert Setup"};
  menuLib[1][1]= {""};
 
  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  while ((keepMenu==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 2);
  
    if (dbgMode>=1){Serial.println(fctName+F("|posMenuNew=")+String(posMenuNew));}
    
    // Si la touche Valide est pressee, alors on passe dans le menu suivant
    if (posMenuNew == -1){
      switch (posMenu){
        case 0 : //Lancer le mode Step
          //On va parcourir les Steps du mode expert et on va appliquer les consignes sur la durée demandée
          //La touche Back va arreter la chauffe
          //La touche Up va permettre de changer les temps avec sauvegarde dans l'EEPROM
          i=0;
          keepStepping=1;
          while (i<expertModeNbStep && keepStepping==1){
            returnedValue = warmingMenuExpert(i);
            if (returnedValue == -2){
              //La touche a été pressée, alors on sort sans aller au bout
              keepStepping=0;
            }
            i++;
          }
          break;
        case 1 : //Config des step
          expertModeSetup();
          break;
      }

      posMenuNew=posMenu;
    }
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (posMenuNew ==-2){
      keepMenu=0;;
      posMenuNew=posMenu;
    }
    
    posMenu=posMenuNew;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(menuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[posMenu][1]);
  }
}



// Fonction de setup en mode expert
// IN : N/A
// OUT : N/A
void expertModeSetup(){
  String fctName="expertModeSetup";  
  int posMenu=0;
  int posMenuNew=0;
  bool keepMenu=1;


  //Menu Construction
  String menuLib[4][2];
  menuLib[0][0]= "Nb Steps";
  menuLib[0][1]= String(expertModeNbStep);
  menuLib[1][0]= "Step1";
  menuLib[1][1]= String(padding(expertModeStepLength[0],2))+ "min F=" + String(padding(expertModeConsigneFront[0],2)) + " R=" + String(padding(expertModeConsigneRear[0], 2));
  menuLib[2][0]= "Step2";
  menuLib[2][1]= String(padding(expertModeStepLength[1],2))+ "min F=" + String(padding(expertModeConsigneFront[1],2)) + " R=" + String(padding(expertModeConsigneRear[1], 2));
  menuLib[3][0]= "Step3";
  menuLib[3][1]= String(padding(expertModeStepLength[2],2))+ "min F=" + String(padding(expertModeConsigneFront[2],2)) + " R=" + String(padding(expertModeConsigneRear[2], 2));
  
  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  if (expertModeStepLength[posMenu]> 60){
      //affichage logo infini
      lcd.setCursor(0,1);
      lcd.write(byte(1));
      lcd.setCursor(1,1);
      lcd.write(byte(1));
    }

    while ((keepMenu==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, expertModeNbStep+1);
        
    // Si la touche Valide est pressee, alors on passe dans le menu suivant
    if (posMenuNew == -1){
      if (posMenu==0){
        // Config du nb de Step
        expertModeSetupNbStep();
        menuLib[0][1]= String(expertModeNbStep);
      }
      else {
        // Config du step ciblé
        expertModeSetupStep(posMenu-1);
        menuLib[posMenu][1]= String(padding(expertModeStepLength[posMenu],2))+F("min F=") + String(padding(expertModeConsigneFront[posMenu],2)) + F(" R=") + String(padding(expertModeConsigneRear[posMenu], 2));
      }

      posMenuNew=posMenu;
    }
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (posMenuNew ==-2){
      keepMenu=0;;
      posMenuNew=posMenu;
    }
    
    posMenu=posMenuNew;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(menuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[posMenu][1]);
    
    if (expertModeStepLength[posMenu]> 60){
      //affichage logo infini
      lcd.setCursor(0,1);
      lcd.write(byte(1));
      lcd.setCursor(1,1);
      lcd.write(byte(1));
    }
  }
}

// Fonction du config du nombre de Step mode Expert
// IN : N/A
// OUT : N/A
void expertModeSetupNbStep(){
  String fctName="expertModeSetupNbStep";
  
  int posMenuNew=0;
  bool keepMenu=1;

  lcd.setCursor(0,1);
  lcd.cursor();
  lcd.blink();
    
  while(keepMenu==1){
    posMenuNew = readBtn(expertModeNbStep, 1, 4);
    if (posMenuNew ==-1 || posMenuNew==-2){
      posMenuNew=expertModeNbStep;
      keepMenu=0;
    }
    else{
      expertModeNbStep=posMenuNew;
    }
    lcd.print(String(expertModeNbStep));
    lcd.setCursor(0,1);
  }

  //Avant de sortir, on enregistre dans l'EEPROM
  //writeEEPROM(eeprom, screenContrastValEepromAddress, screenContrastVal);
  
  lcd.noCursor();
  lcd.noBlink();    
}

// Fonction du config d'un Step mode Expert
// IN : N/A
// OUT : N/A
void expertModeSetupStep(int monStep){
  String fctName="expertModeSetupStep";

  int posMenu=0;
  int posMenuNew=0;
  int itemEnCours=0; //0 : c'est le délai, 1 : c'est la temp avant, 2: c'est la temps arrière
  int itemScreenPos[3]={0, 8, 13};
  bool keepMenu=1;

  lcd.cursor();
  lcd.blink();
    
  while(keepMenu==1){
    //On positionne le cursor au bon endroit
    lcd.setCursor(itemScreenPos[itemEnCours],1);
    
    //la valeur max dépend de l'item qu'on est en train de paramatrer
    // Pour la durée du Step : 5 à 60 et OFF
    // Pour les température : 20 à 75

    if (itemEnCours ==0 ){
        posMenu = expertModeStepLength[monStep];
        posMenuNew = readBtn(posMenu, 2, 62);
    }
    else {
      if (itemEnCours ==1){
        posMenu = expertModeConsigneFront[monStep];
        posMenuNew = readBtn(posMenu, 20, 75);
      }
      else {
        posMenu = expertModeConsigneRear[monStep];
        posMenuNew = readBtn(posMenu, 20, 75);
      }
    }
    
    if (posMenuNew ==-2 ){
      //si back est pressé on sort
      keepMenu=0;
    } else {
      if (posMenuNew==-1){
        // si val est pressé on passe à la valeur suivante et on déplace le curseur
        itemEnCours++;
      }
      else{
        // Sinon on met à jour la valeur en cours
        // dans la variable et sur le LCD
        // specificité pour délai qui peut être mis à off si = 61min
        if (itemEnCours ==0 ){
          expertModeStepLength[monStep]=posMenuNew;   
        }
        else {
          expertModeConsigneFront[monStep]=posMenuNew;
        }    
      }
    }

    //MAJ affichage
    lcd.setCursor(0,1);
      lcd.print(String(padding(expertModeStepLength[monStep],2))+F("min F=") + String(padding(expertModeConsigneFront[monStep],2)) + F(" R=") + String(padding(expertModeConsigneRear[monStep], 2)));
    if (expertModeStepLength[monStep]> 60){
      //affichage logo infini
      lcd.setCursor(0,1);
      lcd.write(byte(1));
      lcd.setCursor(1,1);
      lcd.write(byte(1));
    }
  }

  //Avant de sortir, on enregistre dans l'EEPROM
  //writeEEPROM(eeprom, screenContrastValEepromAddress, screenContrastVal);
  
  lcd.noCursor();
  lcd.noBlink();  
}

// Fonction de paramétrage du contrast
// IN : N/A
// OUT : N/A
void contrastConfig(){
  String fctName="contrastConfig";
    
  int posMenuNew=0;
  bool keepMenu=1;

  lcd.setCursor(0,1);
  lcd.cursor();
  lcd.blink();
    
  while(keepMenu==1){
    posMenuNew = readBtn(screenContrastVal, 0, 5);
    if (posMenuNew ==-1 || posMenuNew==-2){
      posMenuNew=screenContrastVal;
      keepMenu=0;
    }
    else{
      screenContrastVal=posMenuNew;
    }

    //On met à jour le contraste
    analogWrite(screenContrastPin, screenContrastLst[screenContrastVal]);
  
    lcd.setCursor(0,1);
    lcd.print(F("                "));
    lcd.setCursor(0,1);
    lcd.print(String(screenContrastLib[screenContrastVal]));
    lcd.setCursor(0,1);
    lcd.cursor();
    lcd.blink();    
  }

  //Avant de sortir, on enregistre dans l'EEPROM
  writeEEPROM(eeprom, screenContrastValEepromAddress, screenContrastVal);
  
  lcd.noCursor();
  lcd.noBlink();    
}



// Fonction de paramétrage des valeurs de coupure automatique sur délai
// IN : N/A
// OUT : N/A
void autoCutConfig(){
  String fctName="autoCutConfig";
    
  int posMenuNew=0;
  bool keepMenu=1;

  lcd.setCursor(0,1);
  lcd.cursor();
  lcd.blink();
    
  while(keepMenu==1){
    posMenuNew = readBtn(autoCutVal, 0, 5);
    if (posMenuNew ==-1 || posMenuNew==-2){
      posMenuNew=autoCutVal;
      keepMenu=0;
    }
    else{
      autoCutVal=posMenuNew;
    }
  
  
    lcd.setCursor(0,1);
    lcd.print(F("                "));
    lcd.setCursor(0,1);
    lcd.print(String(autoCutLib[autoCutVal]));
    lcd.setCursor(0,1);
    lcd.cursor();
    lcd.blink();    
  }


  //Avant de sortir, on enregistre dans l'EEPROM
  writeEEPROM(eeprom, autoCutValEepromAddress, autoCutVal);
  
  lcd.noCursor();
  lcd.noBlink();
}

// Fonction de paramétrage des corrections de températures
// IN : N/A
// OUT : N/A
void correctionTempConfig(){
  String fctName="correctionTempConfig";

  int posMenu=0;
  int posMenuNew=0;

  int posSousMenu=0;
  int posSousMenuNew=0;
  bool keepMenu=1;
  bool keepSetuping=1;

  int sensorCurrent=0;
  int chauffeCurrent=0;

  int configBaseTemp=50;
    
  while(keepMenu==1){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(couvList[posMenu]+ F(" Adjust Temp"));
    lcd.setCursor(0,1);
    lcd.print(F("                "));
    
    posMenuNew = readBtn(posMenu, 0, 4);
    if (posMenuNew ==-1){
      // La on va faire le setup
      // On commence à chauffer jusqu'a 50 --> On affiche un menu d'attente
      // Quand on a atteint 50, on propose a l'utilisateur de corriger la température
      lcd.setCursor(0,1);
      lcd.print(F("Please Wait..."));

      switch (posMenu){
        case 0 :
            sensorCurrent=sensorFL;
            chauffeCurrent=chauffeFL;
            break;
        case 1 :
            sensorCurrent=sensorFR;
            chauffeCurrent=chauffeFR;
            break;
        case 2 :
            sensorCurrent=sensorRL;
            chauffeCurrent=chauffeRL;
            break;
        case 3 :
            sensorCurrent=sensorRR;
            chauffeCurrent=chauffeRR;
            break;
      }
      keepSetuping=1;
      posSousMenu=configBaseTemp+correctionTemp[posMenu];
      while (keepSetuping==1){
          temperature[posMenu]=warmingCheckAdjust(sensorCurrent, chauffeCurrent, 50, 0, 0, 17);
          // Si on atteint la température attendue à 10° pres... on affiche la température
          if ( temperature[posMenu]>configBaseTemp-10 ){
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print(couvList[posMenu]+F(" mesured=")+String(temperature[posMenu]));
              lcd.setCursor(0,1);
              lcd.print(String(F("Corrected="))+String(posSousMenu));
              lcd.setCursor(10,1);
              lcd.cursor();
              lcd.blink();    
          }

          //On lit les boutons régulièrement
          posSousMenu = readBtn(posSousMenu, 0, 75);
          
          if (posSousMenu==-1 || posSousMenu==-2){
            // On sort de la conf de cette couv
            digitalWrite(chauffeCurrent, LOW);
            keepSetuping=0;
          }
          else {
            correctionTemp[posMenu]=posSousMenu-configBaseTemp;
          }
      }
    }
    else{
      if (posMenuNew ==-2){
        //On sort du Menu de correction des temp
        keepMenu=0;
      }
      else{
        posMenu=posMenuNew;
      }
    }
    //On enregistre la correction sur l'EEPROM
    writeEEPROM(eeprom, correctionTempEepromAddress[posMenu], correctionTemp[posMenu]);
  }
  
  lcd.clear();
  lcd.noCursor();
  lcd.noBlink();    
} 

// Fonction de rétablissement des setup par default
// IN : N/A
// OUT : N/A
void restoreDefault(){
  String fctName="restoreDefault";

  int posMenuNew=0;   
  bool keepMenu=1;

  while(keepMenu==1){

    posMenuNew = readBtn(0, 0, 0);
    if (posMenuNew ==-1){
      lcd.setCursor(0,1);
      lcd.print(F("...Done..."));
      // les temps par dafaut
      writeEEPROM(eeprom, consigneEepromAddress[0], 50);
      writeEEPROM(eeprom, consigneEepromAddress[1], 50);

      //Contrast et delai
      writeEEPROM(eeprom, autoCutValEepromAddress, 2);
      writeEEPROM(eeprom, screenContrastValEepromAddress, 2);
      // Correction des températures
      writeEEPROM(eeprom, correctionTempEepromAddress[0], 0);
      writeEEPROM(eeprom, correctionTempEepromAddress[1], 0);
      writeEEPROM(eeprom, correctionTempEepromAddress[2], 0);
      writeEEPROM(eeprom, correctionTempEepromAddress[3], 0);

      //Variables pour le mode expert
      writeEEPROM(eeprom, expertModeNbStepEepromAddress, 3);
      writeEEPROM(eeprom, expertModeConsigneFrontEepromAddress[0], 50);
      writeEEPROM(eeprom, expertModeConsigneFrontEepromAddress[1], 55);
      writeEEPROM(eeprom, expertModeConsigneFrontEepromAddress[2], 60);

      writeEEPROM(eeprom, expertModeConsigneRearEepromAddress[0], 50);
      writeEEPROM(eeprom, expertModeConsigneRearEepromAddress[1], 55);
      writeEEPROM(eeprom, expertModeConsigneRearEepromAddress[2], 60);

      writeEEPROM(eeprom, expertModeStepLengthEepromAddress[0], 20);
      writeEEPROM(eeprom, expertModeStepLengthEepromAddress[1], 5);
      writeEEPROM(eeprom, expertModeStepLengthEepromAddress[2], 10);

      // On va aller Lire le contenu de l'EEPROM
      consigne[0] = readEEPROM(eeprom, consigneEepromAddress[0]);
      consigne[1] = readEEPROM(eeprom, consigneEepromAddress[1]);

      autoCutVal = readEEPROM(eeprom, autoCutValEepromAddress);
      screenContrastVal = readEEPROM(eeprom, screenContrastValEepromAddress);  
      correctionTemp[0] = readEEPROM(eeprom, correctionTempEepromAddress[0]);
      correctionTemp[1] = readEEPROM(eeprom, correctionTempEepromAddress[1]);
      correctionTemp[2] = readEEPROM(eeprom, correctionTempEepromAddress[2]);
      correctionTemp[3] = readEEPROM(eeprom, correctionTempEepromAddress[3]);

      expertModeNbStep= readEEPROM(eeprom, expertModeNbStepEepromAddress);

      expertModeConsigneFront[0]= readEEPROM(eeprom, expertModeConsigneFrontEepromAddress[0]);
      expertModeConsigneFront[1]= readEEPROM(eeprom, expertModeConsigneFrontEepromAddress[1]);
      expertModeConsigneFront[2]= readEEPROM(eeprom, expertModeConsigneFrontEepromAddress[2]);

      expertModeConsigneRear[0]= readEEPROM(eeprom, expertModeConsigneRearEepromAddress[0]);
      expertModeConsigneRear[1]= readEEPROM(eeprom, expertModeConsigneRearEepromAddress[1]);
      expertModeConsigneRear[2]= readEEPROM(eeprom, expertModeConsigneRearEepromAddress[2]);

      expertModeStepLength[0]= readEEPROM(eeprom, expertModeStepLengthEepromAddress[0]);
      expertModeStepLength[1]= readEEPROM(eeprom, expertModeStepLengthEepromAddress[1]);
      expertModeStepLength[2]= readEEPROM(eeprom, expertModeStepLengthEepromAddress[2]);
               
      delay(1500);
      keepMenu=0;
    } 
    if (posMenuNew ==-2){
      keepMenu=0;
    } 
  }
  
}

 
  
// Fonction de lecture des boutons de saisie : UP, DOWN, Valid et Back
// IN : Id actuel du Menu, int valeur mini dans ce menu, Nb max d'éléments dans ce menu
// OUT : Nouvel ID de Menu 
//prends la valeur -1 c'est le bouton valider
//prends la valeur -2 c'est le bouton Back
int readBtn(int maPosMenu, int minNbElts, int maxNbElts){
  String fctName="readBtn";
  bool btnPressed=0;
  byte nbBoucle=0;
  int BtnReadVal=0;
 
  // On va boucler ici tant qu'un bouton n'est pas appuyé
  // Mais on va aussi sortir toutes les 10 boucles (~0.5s)
  while (btnPressed ==0 && nbBoucle <5 ){
    // Lecture des boutons appuyés
    BtnReadVal = analogRead (BtnPin);
    
    // > 280 ==> Aucun bouton
    // entre 200 et 280 ==> Down
    // entre 120 et 200   => UP
    // entre 50 et 120 => Valider
    // < 50 ==> Back
    if (dbgMode>=2){Serial.println(fctName+F("|BtnReadVal=")+String(BtnReadVal));}
    

    //Bouton Down
    if (BtnReadVal>=200 && BtnReadVal<280 ){
        btnPressed=1;
        if (maPosMenu == minNbElts){
            maPosMenu=maxNbElts-1;
        }
        else {
            maPosMenu--;
        } 
        delay(200);
    }

    //Bouton Valider
    if (BtnReadVal>=50 && BtnReadVal<120 ){
        btnPressed=1;
        maPosMenu = -1;
        delay(200);
    }

        //Bouton UP
    if (BtnReadVal>=120 && BtnReadVal<200){
       btnPressed=1;
       if (maPosMenu == maxNbElts-1){
          maPosMenu=minNbElts;
       }
       else {
          maPosMenu++;
       }
       delay(200); 
    }


    //Bouton Back
    if (BtnReadVal<50 ){
        btnPressed=1;
        maPosMenu = -2;
        delay(200);
    }
    nbBoucle++;
    delay(100);
  }
  return maPosMenu;
}

// Fonction d'écriture dans l'EEPROM
// IN : adresse de l'EEPROM, l'octet ciblé, l'octet à écrire
// OUT : N/A
void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}

// Fonction d'écriture dans l'EEPROM
// IN : adresse de l'EEPROM, l'octet ciblé
// OUT : l'octet lu
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,1);
 
  if (Wire.available()) rdata = Wire.read();
 
  return rdata;
}

//Pour utiliser la fonction d'économie de RAM dans les print F()
//from http://jeelabs.org/2011/05/22/atmega-memory-use/
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//Fonction de padding des nombres
String padding( int number, byte width ) {
 int currentMax = 10;
 String padded="";
 for (byte i=1; i<width; i++){
   if (number < currentMax) {
     padded=padded+"0";
   }
   currentMax *= 10;
 }
 return padded+number;
}

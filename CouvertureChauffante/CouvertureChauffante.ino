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
int screenContrastLst[5]={200, 150, 100, 50, 10};
String screenContrastLib[5]={"=", "====", "========", "============", "================"};
int screenContrastVal = 2;
unsigned int screenContrastValEepromAddress = 5;

// Conf du Delay en secondes 
int autoCutLst[5]={-1, 10, 3600, 7200, 14400};
String autoCutLib[5]={"OFF", "30min", "1h", "2h", "4h"};
int autoCutVal = 3;
unsigned int autoCutValEepromAddress = 4;

//conf des libelles
String couvList[4]={"FL", "FR", "RL", "RR"};

//Conf tes températures
unsigned int consigneEepromAddress[2] = {0,1};
int consigne[2]={50,50};
int correctionTemp[4]={0,0,0,0};
int temperature[4]={0,0,0,0};
unsigned int correctionTempEepromAddress[4] = {6,7,8,9};

//Conf de l'expertMode
int expertModeNbStep=3;
int expertModeConsigneFront[3]={10, 20, 30};
int expertModeConsigneRear[3]={15, 25, 35};
int expertModeStepLength[3]={5,10,15};

//Initialisation des capteurs de temp
int sensorFL=A1;
int sensorFR=A0;
int sensorRL=A2;
int sensorRR=A3;
int B=3975;  // Alors ca je ne sais pas d'ou ca sort :-)

//Initialisation des fils resistifs
int chauffeFL=9;
int chauffeFR=10;
int chauffeRL=7;
int chauffeRR=8;

// Création du caractère Flèche
byte arrow[8] = {
  B00000,
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
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

  //Mise jour du contrast
  analogWrite(screenContrastPin, screenContrastLst[screenContrastVal]);
  // set up the LCD's number of columns and rows and contrast Init
  lcd.begin(16, 2);
  lcd.print(F("www.ae-rc.com"));
  delay (1500);

  // Initialistion du caractère créé
  lcd.createChar(0, arrow);
  
}

void loop() {
  mainMenu();
}

void mainMenu(){
  String fctName="mainMenu";

  int posMenu=0;
  int posMenuNew=0;
  
  //Menu Construction
  String mainMenuLib[3][2];
  mainMenuLib[0][0]= {"1.Quick Warming"};
  mainMenuLib[0][1]= {""};
  mainMenuLib[1][0]= {"2.Expert Mode"};
  mainMenuLib[1][1]= {""};
  mainMenuLib[2][0]= {"3.Setup"};
  mainMenuLib[2][1]= {""};

  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(mainMenuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(mainMenuLib[posMenu][1]);

  while ((1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 3);
  
    if (dbgMode>=1){Serial.println(fctName+F("|")+String(freeRam())+F("|posMenuNew=")+String(posMenuNew));}
    
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
    lcd.print(mainMenuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(mainMenuLib[posMenu][1]);
     
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
  
  //Menu Construction
  String menuLib[1][2];
  int startWarmingTime=millis();
      
  //On initialise l'affichage
  menuLib[0][0]= {"F>"+String(consigne[0])+ " L=00  R=00"};
  menuLib[0][1]= {"R>"+String(consigne[1])+ " L=00  R=00"};
  posMenu=posMenuNew;
  lcd.clear();
  lcd.noCursor();
  lcd.noBlink();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);
  
  while ((keepWarming==1)){
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture
    //1. read les 4 temps
    //2. ajuster la tension
    //3. Ajuster l'affichage
    temperature[0]=warmingCheckAdjust(sensorFL, chauffeFL, consigne[0], correctionTemp[0], 0, 7);
    temperature[1]=warmingCheckAdjust(sensorFR, chauffeFR, consigne[0], correctionTemp[1], 0, 13);
    temperature[2]=warmingCheckAdjust(sensorRL, chauffeRL, consigne[1], correctionTemp[2], 1, 7);
    temperature[3]=warmingCheckAdjust(sensorRR, chauffeRR, consigne[1], correctionTemp[3], 1, 13);
    
    //On met à jour l'affichage
    menuLib[0][0]= {"F>"+String(consigne[0])+ " L="+String((round(temperature[0])))+"  R="+String((round(temperature[1])))};
    menuLib[0][1]= {"R>"+String(consigne[1])+ " L="+String((round(temperature[2])))+"  R="+String((round(temperature[3])))};
    lcd.setCursor(7,0);
    lcd.print(String(round(temperature[0])));
    lcd.setCursor(13,0);
    lcd.print(String(round(temperature[1])));
    lcd.setCursor(7,1);
    lcd.print(String(round(temperature[2])));
    lcd.setCursor(13,1);
    lcd.print(String(round(temperature[2])));

    posMenu=posMenuNew;
    
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 2);
  
    if (dbgMode>=1){Serial.println(fctName+F("|posMenuNew=")+String(posMenuNew));}
    
    // Si la touche Valide est pressee, alors on passe dans l'écran de réglage des temps
    // Mais avant on coupe la chauffe
    if (posMenuNew == -1){
      digitalWrite(chauffeFL, LOW);
      digitalWrite(chauffeFR, LOW);
      digitalWrite(chauffeRL, LOW);
      digitalWrite(chauffeRR, LOW);
      warmingSetup("");
      posMenuNew=posMenu;
    }

    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (posMenuNew ==-2){
      keepWarming=0;
      posMenuNew=posMenu;
    }

     // Si une touche a été pressée
     // On affiche depuis combien de temps ca chauffe pendant 2.5s
     // Puis on ré-affiche les temperatures
    if (posMenuNew != posMenu ){
      lcd.clear();
      lcd.noCursor();
      lcd.noBlink();
      lcd.setCursor(0,0);
      lcd.print(F("Warming time:"));
      lcd.setCursor(0,1);
      
      lcd.print((String(round(millis()-startWarmingTime)/1000/60))+"s");
      delay (2500);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(menuLib[posMenu][0]);
      lcd.setCursor(0,1);
      lcd.print(menuLib[posMenu][1]);
    }
    
    // On Check si on a pas atteint la fin du delay
    if (dbgMode>=2){Serial.println(fctName+F("|startWarmingTime=")+String(startWarmingTime)+F("|millis=")+String(millis()));}
    if ((millis()-startWarmingTime)/1000 > autoCutLst[autoCutVal]){
      keepWarming=0;
    }

     
  }
  //On coupe la chauffe
  digitalWrite(chauffeFL, LOW);
  digitalWrite(chauffeFR, LOW);
  digitalWrite(chauffeRL, LOW);
  digitalWrite(chauffeRR, LOW);
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
  if (dbgMode>=1){Serial.print(fctName+F("|")+String(freeRam())+F("|position=")+String(ligneCurrent)+F("/")+String(colonneCurrent)+F(" |mesured=")+String(transformedValue)+F(" |consigne=")+String(consigneCurrent)+F(" |correction=")+String(tempCorrectionCurrent));}

  //On check si on doit couper la chauffe
  //3 mode différents :
  //  - si je suis au dessus : je coupe
  //  - si je suis largement en dessous : j'allume
  //  - Si je suis à 2° près en dessous : j'allume pour 0.5s
  if (transformedValue+tempCorrectionCurrent>=consigneCurrent) {
      digitalWrite(sensorChauffe, LOW); 
      lcd.setCursor(colonneCurrent+2,ligneCurrent);
      lcd.print(F(" ")); 
      if (dbgMode>=1){Serial.println(F("| --> OFF"));}
  }
  else{
      if (transformedValue+tempCorrectionCurrent<consigneCurrent-2){
          digitalWrite(sensorChauffe, HIGH);
          lcd.setCursor(colonneCurrent+2,ligneCurrent);
          lcd.write(byte(0));  //Affichage de la flèche
          if (dbgMode>=1){Serial.println(F("| --> ON"));}
      }
      else
      {
          digitalWrite(sensorChauffe, HIGH);
          lcd.setCursor(colonneCurrent+2,ligneCurrent);
          lcd.write(byte(0));  //Affichage de la flèche
          if (dbgMode>=1){Serial.println(F("| --> ON_short"));}
          delay(500);
          digitalWrite(sensorChauffe, LOW);   
          lcd.print(F(" "));    
      }
  }
  return transformedValue+tempCorrectionCurrent;
}

// Fonction qui permet de régler les consignes de température
// IN : N/A
// OUT : N/A
void warmingSetup(String stepLabel){
  String fctName="warmingSetupMenu";

  int posMenu=0;
  int posMenuNew=0;
  
  int cursorPos[2]={4,14};
  int cursorPosCurrent=0;
  
  bool keepSetuping=1;
  
  //Menu Construction
  String menuLib[1][2];
  menuLib[0][0]= {"T° Setup : "+stepLabel};
  menuLib[0][1]= {"FT="+String(consigne[0])+"     RR="+String(consigne[1])};

  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  //On positionne curseur on bon endroit et on le fait clignoter
  lcd.setCursor(cursorPos[posMenu],1);
  lcd.cursor();
  lcd.blink();


  while ((keepSetuping==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(consigne[cursorPosCurrent], 10, 75);
  
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
    }
    else {
     consigne[1]= posMenuNew;
      
    }
    
    // On met à jour le texte et on affiche
    menuLib[0][1]= {"FT="+String(consigne[0])+"     RR="+String(consigne[1])};

    lcd.clear();
    lcd.noCursor();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print(menuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[posMenu][1]);
 
    //On positionne curseur on bon endroit et on le fait clignoter
    lcd.setCursor(cursorPos[cursorPosCurrent],1);
    lcd.cursor();
    lcd.blink();   
  }

  //Avant de sortir on enregistre les 4 consignes dans l'EEPROM
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
  bool keepMenu=1;
  //Menu Construction
  String menuLib[2][2];
  menuLib[0][0]= {"1.Start Warming"};
  menuLib[0][1]= {""};
  menuLib[1][0]= {"2.Setup Mode"};
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
        case 0 : // Config du contrast
          expertModeWarming();
          break;
        case 1 : // auto cut-off delay
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


// Fonction de chauffe en mode expert
// IN : N/A
// OUT : N/A
void expertModeWarming(){
  String fctName="expertModeWarming";
  if (dbgMode>=1){Serial.println(fctName);}
 
}

// Fonction de chauffe en mode expert
// IN : N/A
// OUT : N/A
void expertModeSetup(){
  String fctName="expertModeSetup";
  if (dbgMode>=1){Serial.println(fctName);}
  
  int posMenu=0;
  int posMenuNew=0;
  bool keepMenu=1;


  //Menu Construction
  String menuLib[4][2];
  menuLib[0][0]= "1.Nb Steps";
  menuLib[0][1]= String(expertModeNbStep);
  menuLib[1][0]= "2.Step1";
  menuLib[1][1]= String(expertModeStepLength[0])+"min F=" + String(expertModeConsigneFront[0]) + " R=" + String(expertModeConsigneRear[0]);
  menuLib[2][0]= "3.Step2";
  menuLib[2][1]= String(expertModeStepLength[1])+"min F=" + String(expertModeConsigneFront[1]) + " R=" + String(expertModeConsigneRear[1]);
  menuLib[3][0]= "4.Step3";
  menuLib[3][1]= String(expertModeStepLength[2])+"min F=" + String(expertModeConsigneFront[2]) + " R=" + String(expertModeConsigneRear[2]);
  
  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);
  delay (1000);
    while ((keepMenu==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, expertModeNbStep+1);
        
    // Si la touche Valide est pressee, alors on passe dans le menu suivant
    if (posMenuNew == -1){
      if (posMenu==0){
        // Config du nb de Step
        expertModeSetupNbStep();
      }
      else {
        // Config du step ciblé
        expertModeSetupStep(posMenu-1);
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
        posMenuNew = readBtn(posMenu, 5, 61);
    }
    else {
        posMenu = expertModeConsigneFront[monStep];
        posMenuNew = readBtn(posMenu, 20, 75);
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
    lcd.print(String(expertModeStepLength[monStep])+F("min F=") + String(expertModeConsigneFront[0]) + F(" R=") + String(expertModeConsigneRear[0]));
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
    lcd.print(couvList[posMenu]+ " Adjust Temp");
    lcd.setCursor(0,1);
    lcd.print(F("                "));
    
    posMenuNew = readBtn(posMenu, 0, 3);
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
        posMenuNew=posMenu;
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

    posMenuNew = readBtn(autoCutVal, 0, 0);
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

      // On va aller Lire le contenu de l'EEPROM
      consigne[0] = readEEPROM(eeprom, consigneEepromAddress[0]);
      consigne[1] = readEEPROM(eeprom, consigneEepromAddress[1]);

      autoCutVal = readEEPROM(eeprom, autoCutValEepromAddress);
      screenContrastVal = readEEPROM(eeprom, screenContrastValEepromAddress);  
      correctionTemp[0] = readEEPROM(eeprom, correctionTempEepromAddress[0]);
      correctionTemp[1] = readEEPROM(eeprom, correctionTempEepromAddress[1]);
      correctionTemp[2] = readEEPROM(eeprom, correctionTempEepromAddress[2]);
      correctionTemp[3] = readEEPROM(eeprom, correctionTempEepromAddress[3]);          
               
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
  int nbBoucle=0;
  int BtnReadVal=0;
 
  // On va boucler ici tant qu'un bouton n'est pas appuyé
  // Mais on va aussi sortir toutes les 10 boucles (~1s)
  while (btnPressed ==0 && nbBoucle <10 ){
    // Lecture des boutons appuyés
    BtnReadVal = analogRead (BtnPin);
    
    // > 280 ==> Aucun bouton
    // entre 200 et 280 ==> Down
    // entre 120 et 200 ==> Valider
    // entre 50 et 120 ==> UP
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
    if (BtnReadVal>=120 && BtnReadVal<200){
        btnPressed=1;
        maPosMenu = -1;
        delay(200);
    }
    
    //Bouton UP
    if (BtnReadVal>=50 && BtnReadVal<120 ){
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


//from http://jeelabs.org/2011/05/22/atmega-memory-use/
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

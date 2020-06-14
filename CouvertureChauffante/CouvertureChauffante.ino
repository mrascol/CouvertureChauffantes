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
int screenContrast=6;
int screenContrastLst[5]={200, 150, 100, 50, 10};
String screenContrastLib[5]={"=", "====", "========", "============", "================"};
int screenContrastVal = 2;
unsigned int screenContrastValEepromAddress = 5;

// Conf du Delay en secondes 
int autoCutLst[5]={-1, 10, 3600, 7200, 14400};
String autoCutLib[5]={"OFF", "30min", "1h", "2h", "4h"};
int autoCutVal = 3;
unsigned int autoCutValEepromAddress = 4;


//Conf tes températures
unsigned int consigneEepromAddress[4] = {0,1,2,3};
int consigne[4]={50,50,50,50};
int temperature[4]={0,0,0,0};

//Initialisation des capteurs de temp
int sensorFL=A0;
int sensorFR=A1;
int sensorRL=A2;
int sensorRR=A3;
int B=3975;  // Alors ca je ne sais pas d'ou ca sort :-)

//Initialisation des fils resistifs
int chauffeFL=9;
int chauffeFR=10;
int chauffeRL=7;
int chauffeRR=8;


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
  pinMode(screenContrast, OUTPUT);

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
  // pour le contrast
  // pour la valeur d'auto cut-off
  // pour les consignes de températures
  consigne[0] = readEEPROM(eeprom, consigneEepromAddress[0]);
  consigne[1] = readEEPROM(eeprom, consigneEepromAddress[1]);
  consigne[2] = readEEPROM(eeprom, consigneEepromAddress[2]);
  consigne[3] = readEEPROM(eeprom, consigneEepromAddress[3]);
  autoCutVal = readEEPROM(eeprom, autoCutValEepromAddress);
  screenContrastVal = readEEPROM(eeprom, screenContrastValEepromAddress);                 

  //Mise jour du contrast
  analogWrite(screenContrast, screenContrastLst[screenContrastVal]);
  // set up the LCD's number of columns and rows and contrast Init
  lcd.begin(16, 2);
  lcd.print("www.ae-rc.com");
  delay (1500);
  
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
  mainMenuLib[0][0]= {"1. Start warming"};
  mainMenuLib[0][1]= {""};
  mainMenuLib[1][0]= {"2. Setup"};
  mainMenuLib[1][1]= {""};
  mainMenuLib[2][0]= {"3. Set Timer"};
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
  
    if (dbgMode>=1){Serial.println(fctName+"|posMenuNew="+String(posMenuNew));}
    
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
          setupMenu();
          break;
        case 2 :
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
  menuLib[0][0]= {"FL=xx/"+String(consigne[0])+" FR=xx/"+String(consigne[1])};
  menuLib[0][1]= {"RL=xx/"+String(consigne[2])+" RR=xx/"+String(consigne[3])};

  
  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  //On démarre la chauffe
  digitalWrite(chauffeFL, HIGH);
  
  while ((keepWarming==1)){
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture
    // TODO - voir s'il ne faut pas réduire le timer à 5s dans le fonction ReadBtn
    //1. read les 4 temps
    //2. ajuster la tension
    //3. Ajuster l'affichage
    temperature[0]=warmingCheckAdjust(sensorFL, chauffeFL, consigne[0], 0, 3);
    temperature[1]=warmingCheckAdjust(sensorFR, chauffeFR, consigne[1], 0, 12);
    temperature[2]=warmingCheckAdjust(sensorRL, chauffeRL, consigne[2], 1, 3);
    temperature[3]=warmingCheckAdjust(sensorRR, chauffeRR, consigne[3], 1, 12);

    //On met à jour l'affichage
    menuLib[0][0]= {"FL="+String((round(temperature[0])))+"/"+String(consigne[0])+" FR="+String((round(temperature[1])))+"/"+String(consigne[1])};
    menuLib[0][1]= {"RL="+String((round(temperature[2])))+"/"+String(consigne[2])+" RR="+String((round(temperature[3])))+"/"+String(consigne[3])};
    posMenu=posMenuNew;
    lcd.clear();
    lcd.noCursor();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print(menuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[posMenu][1]);


    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 1);
  
    if (dbgMode>=1){Serial.println(fctName+"|posMenuNew="+String(posMenuNew));}
    
    // Si la touche Valide est pressee, alors on passe dans l'écran de réglage des temps
    // Mais avant on coupe la chauffe
    if (posMenuNew == -1){
      digitalWrite(chauffeFL, LOW);
      digitalWrite(chauffeFR, LOW);
      digitalWrite(chauffeRL, LOW);
      digitalWrite(chauffeRR, LOW);
      warmingSetup();

      //On met à jour l'affichage
      menuLib[0][0]= {"FL=xx/"+String(consigne[0])+" FR=xx/"+String(consigne[1])};
      menuLib[0][1]= {"RL=xx/"+String(consigne[2])+" RR=xx/"+String(consigne[3])};

      posMenuNew=posMenu;
    }
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (posMenuNew ==-2){
      keepWarming=0;;
      posMenuNew=posMenu;
    }
 
    // On Check si on a pas atteint la fin du delay
    if (dbgMode>=2){Serial.println(fctName+"|startWarmingTime="+String(startWarmingTime)+"|millis="+String(millis()));}
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
int warmingCheckAdjust(int sensorCurrent, int sensorChauffe, int consigneCurrent, int ligneCurrent, int colonneCurrent){
  String fctName="warmingCheckAdjust";

  int readValue = 0;
  float resistance;
  float transformedValue;
  
  // Lecture de la température et conversion en °C
  readValue = analogRead(sensorCurrent);
  resistance=(float)(1023-readValue)*10000/readValue; 
  transformedValue=1/(log(resistance/10000)/B+1/298.15)-273.15;
  if (dbgMode>=1){Serial.print(fctName+"|position="+String(ligneCurrent)+"/"+String(colonneCurrent)+" |mesured="+String(transformedValue)+" |consigne="+String(consigneCurrent));}

  //On check si on doit couper la chauffe
  //3 mode différents :
  //  - si je suis au dessus : je coupe
  //  - si je suis largement en dessous : j'allume
  //  - Si je suis à 2° près en dessous : j'allume pour 0.5s
  if (transformedValue>=consigneCurrent) {
      digitalWrite(sensorChauffe, LOW);  
      if (dbgMode>=1){Serial.println("| --> OFF");}

  }
  else{
      if (transformedValue<consigneCurrent-2){
          digitalWrite(sensorChauffe, HIGH);
          if (dbgMode>=1){Serial.println("| --> ON");}
      }
      else
      {
          digitalWrite(sensorChauffe, HIGH);
          if (dbgMode>=1){Serial.println("| --> ON_short");}
          delay(500);
          digitalWrite(sensorChauffe, LOW);
          
        
      }
  }
  return transformedValue;
}

// Fonction qui permet de régler les consignes de température
// IN : N/A
// OUT : N/A
void warmingSetup(){
  String fctName="warmingSetupMenu";

  int posMenu=0;
  int posMenuNew=0;
  
  int cursorPos[2]={4,14};
  int cursorPosCurrent=0;
  
  bool keepSetuping=1;
  
  //Menu Construction
  String menuLib[1][2];
  menuLib[0][0]= {"Temp Setup :"};
  menuLib[0][1]= {"FT="+String(consigne[0])+"     RR="+String(consigne[2])};

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
    posMenuNew = readBtn(consigne[cursorPosCurrent*2], 10, 75);
  
    if (dbgMode>=1){Serial.println(fctName+"|posMenuNew="+String(posMenuNew));}
    
    // Si la touche Valide est pressee, alors on passe au réglage suivant
    if (posMenuNew == -1){
      if (cursorPosCurrent==1){
        cursorPosCurrent=0;
      }
      else{
        cursorPosCurrent=1;
      }
      posMenuNew=consigne[cursorPosCurrent*2];
    }
    
    // si la touche back est pressee, alors on revient à l'écran d'avant
    // Mais on sauvegarde qd meme la valeure
    if (posMenuNew==-2){
      posMenuNew=consigne[cursorPosCurrent*2];
      keepSetuping=0;
    }

    // On met à jour la consigne
    if (cursorPosCurrent==0){
     consigne[0]= posMenuNew;
     consigne[1]= posMenuNew; 
    }
    else {
     consigne[2]= posMenuNew;
     consigne[3]= posMenuNew; 
      
    }
    
    // On met à jour le texte et on affiche
    menuLib[0][1]= {"FT="+String(consigne[0])+"     RR="+String(consigne[2])};

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
  writeEEPROM(eeprom, consigneEepromAddress[2], consigne[2]);
  writeEEPROM(eeprom, consigneEepromAddress[3], consigne[3]);
  
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
  menuLib[0][0]= {"1.Default Temps"};
  menuLib[0][1]= {""};
  menuLib[1][0]= {"2.Contrast"};
  menuLib[1][1]= {String(screenContrastLib[screenContrastVal])};
  menuLib[2][0]= {"3.Buzz Power"};
  menuLib[2][1]= {"TODO"};
  menuLib[3][0]= {"4.Auto cut-off Delay"};
  menuLib[3][1]= {String(autoCutLib[autoCutVal])};
  menuLib[4][0]= {"5.Restore Default"};
  menuLib[4][1]= {""};

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
        case 0 : //Conf des températures défault
          warmingSetup();
          break;
        case 1 : // Config du contrast
          contrastConfig();
          menuLib[1][1]= {String(screenContrastLib[screenContrastVal])};
          break;
        case 2 : // Config du Buzz Power
          break;
        case 3 : // auto cut-off delay
          autoCutConfig();
          menuLib[3][1]= {String(autoCutLib[autoCutVal])};
          break;
        case 4 : //Restore Default
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
    analogWrite(screenContrast, screenContrastLst[screenContrastVal]);
    
  
  
    lcd.setCursor(0,1);
    lcd.print("                ");
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
    lcd.print("                ");
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
      writeEEPROM(eeprom, 0, 50);
      writeEEPROM(eeprom, 1, 50);
      writeEEPROM(eeprom, 2, 50);
      writeEEPROM(eeprom, 3, 50);
      writeEEPROM(eeprom, 4, 2);
      writeEEPROM(eeprom, 5, 2);
      lcd.setCursor(0,1);
      lcd.print("...Done...");
      delay(1000);

      // On va aller Lire le contenu de l'EEPROM
      consigne[0] = readEEPROM(eeprom, consigneEepromAddress[0]);
      consigne[1] = readEEPROM(eeprom, consigneEepromAddress[1]);
      consigne[2] = readEEPROM(eeprom, consigneEepromAddress[2]);
      consigne[3] = readEEPROM(eeprom, consigneEepromAddress[3]);
      autoCutVal = readEEPROM(eeprom, autoCutValEepromAddress);
      screenContrastVal = readEEPROM(eeprom, screenContrastValEepromAddress);                 
      lcd.setCursor(0,1);
      lcd.print("...Done...");
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
    // entre 120 et 200 ==> Up
    // entre 50 et 120 ==> Valider
    // < 50 ==> Back
    if (dbgMode>=2){Serial.println(fctName+"|BtnReadVal="+String(BtnReadVal));}
    

    //Bouton Down
    if (BtnReadVal>=200 && BtnReadVal<280 ){
        btnPressed=1;
        if (maPosMenu == minNbElts){
            maPosMenu=maxNbElts-1;
        }
        else {
            maPosMenu--;
        } 
        delay(300);
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
       delay(300); 
    }
    //Bouton Valider
    if (BtnReadVal>=50 && BtnReadVal<120 ){
        btnPressed=1;
        maPosMenu = -1;
        delay(300);
    }

    //Bouton Back
    if (BtnReadVal<50 ){
        btnPressed=1;
        maPosMenu = -2;
        delay(300);
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

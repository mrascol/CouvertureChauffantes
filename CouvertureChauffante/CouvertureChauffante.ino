#include <LiquidCrystal.h>// include the library for LCD 
#include <Wire.h>     // include the library for EEPROM adressing
 
#define eeprom 0x50    //Address of 24LC128 eeprom chip
#define eepromSize 128  //Taille de l'EEPROM - ici 128kbits = 16Ko

// VERSION
String hwVersion="1.0";
String swVersion="1.3";

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
  // Init Serial
  Serial.begin(9600);
  while(!Serial);

  
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


  //Mise jour du contrast
  analogWrite(screenContrastPin, screenContrastLst[screenContrastVal]);
  // set up the LCD's number of columns and rows and contrast Init
  lcd.begin(16, 2);
  lcd.print(F("www.ae-rc.com"));
  lcd.setCursor(0,1);
  lcd.print("HW=" + hwVersion + F("  SW=") + swVersion);
  delay (3000);

  // Initialistion du caractère créé
  lcd.createChar(0, arrow);
  lcd.createChar(1, infini);
  lcd.createChar(2, arrow_small);
  
}

void loop() {
  int posMenu=0;
  int posMenuNew=0;
  
  //Menu Construction
  String mainMenuLib[2];
  mainMenuLib[0]= "1.Quick Warming";
  mainMenuLib[1]= "2.Setup";

  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(mainMenuLib[posMenu]);

  while ((1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 2);
  
   
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
  int posMenu=0;
  int posMenuNew=0;
  bool keepWarming=1;
  byte minutes;
  byte secondes;
  byte cycle;

  //On prends l'heure de démarrage
  unsigned long startWarmingTime=0;
  startWarmingTime=millis();
  

  //On initialise l'affichage
  lcd.clear();
  lcd.noCursor();
  lcd.noBlink();
  lcd.setCursor(0,0);
  lcd.print(F("00m  FL=__ FR=__"));
  lcd.setCursor(0,1);
  lcd.print(F("00s  RL=__ RR=__"));
  cycle=0;
  while ((keepWarming==1)){

    //A chaque cycle, on fait 1 capteur et on lit les boutons
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture.
    switch (cycle){
        case 0 : //FL
            temperature[0]=warmingCheckAdjust(sensorFL, chauffeFL, consigne[0], correctionTemp[0], 0, 8);
            break;
        case 1 : //FR
            temperature[1]=warmingCheckAdjust(sensorFR, chauffeFR, consigne[0], correctionTemp[1], 0, 14);
            break;
        case 2 : //RL
            temperature[2]=warmingCheckAdjust(sensorRL, chauffeRL, consigne[1], correctionTemp[2], 1, 8);
            break;
        case 3 : //RR
            temperature[3]=warmingCheckAdjust(sensorRR, chauffeRR, consigne[1], correctionTemp[3], 1, 14);
            break;
    }
    
    //Mise à jour timer 
    minutes=round(((millis()-startWarmingTime)/1000/60)%99);
    secondes=round(((millis()-startWarmingTime)/1000)%60);
    lcd.setCursor(0,0);
    lcd.print(String(padding(minutes,2)));
    lcd.setCursor(0,1);
    lcd.print(String(padding(secondes,2)));
    
    // On attend qu'un bouton soit pressé
    posMenu=posMenuNew;
    posMenuNew = readBtn(posMenu, 0, 2);
      
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
      lcd.print(F("00m  FL= FR=__"));
      lcd.setCursor(0,1);
      lcd.print(F("00s  RL=__ RR=__"));
      lcd.setCursor(0,0);
      lcd.print(String(padding(minutes,2)) +F("m  FL=") + String((round(temperature[0]))) + F(" FR=") + String((round(temperature[1]))));
      lcd.setCursor(0,1);
      lcd.print(String(padding(secondes,2)) +F("s  RL=") + String((round(temperature[2]))) + F(" RR=") + String((round(temperature[3]))));
      
      posMenuNew=posMenu;
    }

    // Si la touche Valid a été pressée -> on ne fait rien
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (posMenuNew ==-2){
      keepWarming=0;
      posMenuNew=posMenu;
    }

    // On Check si on a pas atteint la fin du delay
    if ((millis()-startWarmingTime)/1000 > autoCutLst[autoCutVal]){
      keepWarming=0;
    }
    cycle=(cycle+1)%4;
  }
  //On coupe la chauffe avant de sortir
  digitalWrite(chauffeFL, LOW);
  digitalWrite(chauffeFR, LOW);
  digitalWrite(chauffeRL, LOW);
  digitalWrite(chauffeRR, LOW);
}



// Fonction qui permet de régler les consignes de température
// IN : le port du Sensor, la consigne pour ce port, la ligne pour l'affichage, la colonne pour l'affichage
// OUT : la nouvelle température mesurée
int warmingCheckAdjust(int sensorCurrent, int sensorChauffe, int consigneCurrent, int tempCorrectionCurrent, int ligneCurrent, int colonneCurrent){
  int readValue = 0;
  float resistance;
  float transformedValue;
  
  // Lecture de la température et conversion en °C
  readValue = analogRead(sensorCurrent);
  resistance=(float)(1023-readValue)*10000/readValue; 
  transformedValue=1/(log(resistance/10000)/B+1/298.15)-273.15;

  //On check si on doit couper la chauffe
  //3 mode différents :
  //  - si je suis au dessus : je coupe
  //  - si je suis largement en dessous : j'allume
  //  - Si je suis à 2° près en dessous : j'allume pour 0.5s
  if (transformedValue+tempCorrectionCurrent>=consigneCurrent) {
      digitalWrite(sensorChauffe, LOW); 
      lcd.setCursor(colonneCurrent-1,ligneCurrent);
      lcd.print(F("=")); 
      lcd.print((int)(transformedValue+tempCorrectionCurrent)); 
  }
  else{
      if (transformedValue+tempCorrectionCurrent<consigneCurrent-2){
          digitalWrite(sensorChauffe, HIGH);
          lcd.setCursor(colonneCurrent-1,ligneCurrent);
          lcd.write(byte(0));  //Affichage de la flèche
          lcd.print((int)(transformedValue+tempCorrectionCurrent));
      }
      else
      {
          digitalWrite(sensorChauffe, HIGH);
          lcd.setCursor(colonneCurrent-1,ligneCurrent);
          lcd.write(byte(0));  //Affichage de la flèche
          delay(500);
          digitalWrite(sensorChauffe, LOW);   
          lcd.setCursor(colonneCurrent-1,ligneCurrent);
          lcd.write(byte(2));
          lcd.print((int)(transformedValue+tempCorrectionCurrent));
      }
  }
  return transformedValue+tempCorrectionCurrent;
}

// Fonction qui permet de régler les consignes de température
// IN : N/A
// OUT : N/A
void warmingSetup(){
  int posMenu=0;
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
    posMenu=consigne[cursorPosCurrent];
    posMenuNew = readBtn(consigne[cursorPosCurrent], 20, 75);
    
    // Si la touche Valide est pressee
    // Si On etait sur l'avant on passe à l'arrière
    // Si On etait à l'arriere on sort
    if (posMenuNew == -1){
      if (cursorPosCurrent==1){
        keepSetuping=0;
      }
      else{
        cursorPosCurrent=1;
      }
      posMenuNew=consigne[cursorPosCurrent];
    }
    
    // si la touche back est pressee, alors on revient à l'écran d'avant
    // Mais on sauvegarde qd meme la valeure
    if (posMenuNew == -2){
      posMenuNew=consigne[cursorPosCurrent];
      keepSetuping=0;
    }

    // On met à jour la consigne - si elle a changé
    // Si on est sur l'avant, on met à jour AV+AR
    // Sinon que l'AR
    if ( posMenuNew != -1 && posMenuNew != -2 && posMenuNew != posMenu ){
      if (cursorPosCurrent==0){
       consigne[0]= posMenuNew;
       consigne[1]= posMenuNew; 
      }
      else {
       consigne[1]= posMenuNew;     
      }
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
  int posMenu=0;
  int posMenuNew=0;
  bool keepMenu=1;
  //Menu Construction
  String menuLib[4][2];
  menuLib[0][0]= {"1.Contrast"};
  menuLib[0][1]= {String(screenContrastLib[screenContrastVal])};
  menuLib[1][0]= {"2.Cut-off Delay"};
  menuLib[1][1]= {String(autoCutLib[autoCutVal])};
  menuLib[2][0]= {"3.Calibrate"};
  menuLib[2][1]= {""};  
  menuLib[3][0]= {"4.Factory Reset"};
  menuLib[3][1]= {""};



  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  while ((keepMenu==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 0, 4);
    
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

// Fonction de paramétrage du contrast
// IN : N/A
// OUT : N/A
void contrastConfig(){
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
  bool btnPressed=0;
  byte nbBoucle=0;
  int BtnReadVal=0;
 
  // On va boucler ici tant qu'un bouton n'est pas appuyé
  // Mais on va aussi sortir toutes les 10 boucles (~0.250s)
  while (btnPressed ==0 && nbBoucle <5 ){
    // Lecture des boutons appuyés
    BtnReadVal = analogRead (BtnPin);
    
    // > 280 ==> Aucun bouton
    // entre 200 et 280 ==> Down
    // entre 120 et 200   => UP
    // entre 50 et 120 => Valider
    // < 50 ==> Back

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
    delay(50);
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

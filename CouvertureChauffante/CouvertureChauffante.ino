// Screen Library
#include <LiquidCrystal.h>// include the library for LCD 
#include <Wire.h>     // include the library for EEPROM adressing

//Nécessaire écriture EEPROM
#include <EEPROM.h>

// VERSION
const String hwVersion="1.0";
const String swVersion="1.7";

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const byte  rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize Bouton
const int BtnPin = A6;
bool upPressed=false;
bool dwnPressed=false;
bool bckPressed=false;
bool valPressed=false;

// Conf du contrast
const byte screenContrastPin=6;

// Conf du Delay en secondes 
const int autoCutLst[5]={-1, 10, 3600, 7200, 14400};
const String autoCutLib[5]={"OFF", "30min", "1h", "2h", "4h"};
byte autoCutVal = 3;


//conf des libelles
const String couvList[4]={"FL", "FR", "RL", "RR"};

//Conf des températures
byte consigne[2]={50,50};
float temperature[4]={0,0,0,0};
float temperaturePrev[4]={0,0,0,0};

//Initialisation des capteurs de temp
const int sensorFL=A1;
const int sensorFR=A0;
const int sensorRL=A3;
const int sensorRR=A2;
const int B=3975;  // Alors ca je ne sais pas d'ou ca sort :-)

//Initialisation des fils resistifs
const byte chauffeFL=9;
const byte chauffeFR=10;
const byte chauffeRL=7;
const byte chauffeRR=8;

//Config de l'adresse EEPROM
template< typename T, size_t N > size_t ArraySize (T (&) [N]){ return N; }

// Création du caractère Flèche
const byte arrow[8] = {
  B00000,
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100
};

const byte arrow_small[8] = {
  B00000,
  B00000,
  B00000,
  B00100,
  B01110,
  B10101,
  B00100
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

  // On va aller Lire le contenu de l'EEPROM
  // pour les consignes de températures
  // pour la valeur d'auto cut-off
  // Les corrections de températures
  //On stock des int, qui font donc 2 octets, donc on écrit tous les 2 octets
  EEPROM.get(0, consigne[0]);
  EEPROM.get(2, consigne[1]);
  EEPROM.get(4, autoCutVal);

  //Mise jour du contrast
  analogWrite(screenContrastPin, 100);
  // set up the LCD's number of columns and rows and contrast Init
  lcd.begin(16, 2);
  lcd.print(F("www.ae-rc.com"));
  lcd.setCursor(0,1);
  lcd.print("HW=" + hwVersion + F("  SW=") + swVersion);

  delay (3000);

  // Initialistion du caractère créé
  lcd.createChar(0, arrow);
  lcd.createChar(2, arrow_small);
}

void loop() {
  byte posMenu=0;
  const String mainMenuLib[2]={"1.Quick Warming", "2.Setup"};

  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(mainMenuLib[posMenu]);

  while ((1)){
    // On attend qu'un bouton soit pressé
    readBtn();
    if (upPressed == true){
      upPressed=false;
      posMenu = (posMenu+1)%2;
    }
    if (dwnPressed == true){
      dwnPressed=false;
      if (posMenu == 0){posMenu =1;}else{posMenu=0;}
    }

    //Si la touche Valid est pressee, on descend dans le bon menu
    if (valPressed == true){
      valPressed=false;
      switch (posMenu){
        case 0 :
          warmingMenu();
          break;
        case 1 :
          setupMenu();
          break;
      }
    }

    if (bckPressed == true){
      bckPressed=false;
    }
     
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
  byte posMenu=0;
  bool keepWarming=true;
  byte minutes;
  byte secondes;
  byte cycle;
  bool hideTemp=false;

  //On prends l'heure de démarrage
  unsigned long startWarmingTime=0;
  startWarmingTime=millis();
  
  //On initialise l'affichage
  lcd.clear();
  lcd.noCursor();
  lcd.noBlink();
  lcd.setCursor(0,0);
  lcd.print(String(F("00m  FL=")) + String(consigne[0]) + F(" FR=")+ String(consigne[0]));
  lcd.setCursor(0,1);
  lcd.print(String(F("00s  RL=")) + String(consigne[1]) + F(" RR=")+ String(consigne[1]));
  cycle=0;
  while ((keepWarming==true)){

    //A chaque cycle, on fait 1 capteur et on lit les boutons
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture.
    //si la température est déconnante <-10 ou >100 On coupe tout
    switch (cycle){
        case 0 : //FL
            warmingCheckAdjust(sensorFL, chauffeFL, 0, consigne[0], 0, 8, hideTemp);
            break;
        case 1 : //FR
            warmingCheckAdjust(sensorFR, chauffeFR, 1, consigne[0], 0, 14, hideTemp);
            break;
        case 2 : //RL
            warmingCheckAdjust(sensorRL, chauffeRL, 2, consigne[1], 1, 8, hideTemp);
            break;
        case 3 : //RR
            warmingCheckAdjust(sensorRR, chauffeRR, 3, consigne[1], 1, 14, hideTemp);
            break;
    }
    
    //Mise à jour timer 
    minutes=round(((millis()-startWarmingTime)/1000/60)%99);
    secondes=round(((millis()-startWarmingTime)/1000)%60);
    lcd.setCursor(0,0);
    lcd.print(padding(minutes,2));
    lcd.setCursor(0,1);
    lcd.print(padding(secondes,2));
    
    // On attend qu'un bouton soit pressé
    readBtn();
      
    // Si la touche UP ou DOWN est pressee, alors on passe dans l'écran de réglage des temps
    // Mais avant on coupe la chauffe
    if (upPressed == true  || dwnPressed == true){
      upPressed=false;
      dwnPressed=false;
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
      lcd.print(padding(minutes,2) +F("m  FL=") + String((round(temperature[0]))) + F(" FR=") + String((round(temperature[1]))));
      lcd.setCursor(0,1);
      lcd.print(padding(secondes,2) +F("s  RL=") + String((round(temperature[2]))) + F(" RR=") + String((round(temperature[3]))));
    }

    // Si la touche Valid a été pressée -> on masque les temps
    if (valPressed == true){
      valPressed =false;
      hideTemp!=hideTemp;

    }
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (bckPressed == true){
      bckPressed =false;
      keepWarming=false;
    }

    // On Check si on a pas atteint la fin du delay
    if ((millis()-startWarmingTime)/1000 > autoCutLst[autoCutVal]){
      keepWarming=false;
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
void warmingCheckAdjust(int sensorCurrent, byte sensorChauffe, byte rang, byte consigneCurrent, byte ligneCurrent, byte colonneCurrent, bool hideTemp){

  //lecture de la temp
  temperaturePrev[rang]=temperature[rang];
  temperature[rang]=readTemp(sensorCurrent);

  //On check si on doit couper la chauffe
  //3 mode différents :
  //  - si je suis largement au dessus ou dans des temps déconnantes: je coupe
  //  - si je suis largement en dessous : j'allume
  //  - Si je suis à 2° près en dessous : j'allume pour 0.5s
  //  - Si je suis à 2° près en dessus et que la tendance est à descendre : j'allume pour 0.5s

  //Largement au dessus ou Tendance à la hausse à consigne et 2° pres au dessus
  if ((temperature[rang]>consigneCurrent + 2 ) ||  (temperature[rang]>=consigneCurrent-2 && temperature[rang] > temperaturePrev[rang]) || temperature[rang] < -10 || temperature[rang] > 100 ) {  
      digitalWrite(sensorChauffe, LOW); 
      lcd.setCursor(colonneCurrent-1,ligneCurrent);
      lcd.print(F("=")); 
  } 

  // Largement en dessous
  if (temperature[rang]<consigneCurrent-1){  
      digitalWrite(sensorChauffe, HIGH);
      lcd.setCursor(colonneCurrent-1,ligneCurrent);
      lcd.write(byte(0));  //Affichage de la flèche
  }

  if (temperature[rang]>=consigneCurrent-1 && temperature[rang]<=consigneCurrent + 2 && temperature[rang] < temperaturePrev[rang]){
          digitalWrite(sensorChauffe, HIGH);
          lcd.setCursor(colonneCurrent-1,ligneCurrent);
          lcd.write(byte(0));  //Affichage de la flèche
          delay((int)((consigneCurrent+2-temperature[rang])*300));
          digitalWrite(sensorChauffe, LOW);
          lcd.setCursor(colonneCurrent-1,ligneCurrent);
          lcd.write(byte(2));                         
  }

  //J'affiche la temperature ou je la cache en fonction du mode
  lcd.setCursor(colonneCurrent,ligneCurrent);
  if ( hideTemp == false && temperature[rang] > -10 && temperature[rang] < 100){
    if (temperature[rang]>=consigneCurrent-3 && temperature[rang]<=consigneCurrent+3){
       lcd.print(String(consigneCurrent));
    }
    else {
        lcd.print((String(round(temperature[rang]))));  
    }
  }
  else 
  {
    lcd.print(F("__"));  
  }
}

// Fonction qui permet de lire une température
// IN : N/A
// OUT : N/A
float readTemp(int sensorCurrent){
  int readValue = 0;
  float resistance;
  float tempMesured;
  
  // Lecture de la température et conversion en °C
  readValue = analogRead(sensorCurrent);
  resistance=(float)(1023-readValue)*10000/readValue; 
  tempMesured=1/(log(resistance/10000)/B+1/298.15)-273.15;

  return tempMesured;
}

// Fonction qui permet de régler les consignes de température
// IN : N/A
// OUT : N/A
void warmingSetup(){
  byte currentTemp=0;
  byte cursorPos[2]={4,14};
  byte cursorPosCurrent=0;
  bool keepSetuping=true;

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

  while ((keepSetuping==true)){
    // On attend qu'un bouton soit pressé
    currentTemp=consigne[cursorPosCurrent];
    readBtn();
    
    // Si la touche Valide est pressee
    // Si On etait sur l'avant on passe à l'arrière
    // Si On etait à l'arriere on sort
    if (valPressed == true){
      valPressed=false;
      if (cursorPosCurrent==1){
        keepSetuping=false;
      }
      else{
        cursorPosCurrent=1;
      }
      currentTemp=consigne[cursorPosCurrent];
    }
    
    // si la touche back est pressee, alors on revient à l'écran d'avant
    // Mais on sauvegarde qd meme la valeure
    if (bckPressed == true){
      bckPressed =false;
      keepSetuping=false;
    }

    // On met à jour la consigne - si elle a changé
    // Si on est sur l'avant, on met à jour AV+AR
    // Sinon que l'AR
    if ( upPressed==true || dwnPressed ==true){
      if (upPressed==true){
        upPressed =false;
        currentTemp= (currentTemp+1)%75;
      }
      else{
        dwnPressed =false;
        if (currentTemp == 0){currentTemp =75;}else{currentTemp=currentTemp-1;}
      }
      
      if (cursorPosCurrent==0){
       consigne[0]= currentTemp;
       consigne[1]= currentTemp; 
      }
      else {
       consigne[1]= currentTemp;     
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
  EEPROM.put(0, consigne[0]);
  EEPROM.put(2, consigne[1]);
  
  lcd.noCursor();
  lcd.noBlink();
}


// Fonction du Menu de paramétrage
// IN : N/A
// OUT : N/A
void setupMenu(){
  byte posMenu=0;
  bool keepMenu=true;
  //Menu Construction
  String menuLib[2][2];
  menuLib[0][0]= {"1.Cut-off Delay"};
  menuLib[0][1]= {String(autoCutLib[autoCutVal])};
  menuLib[1][0]= {"2.Factory Reset"};
  menuLib[1][1]= {""};


  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  while ((keepMenu==true)){
    // On attend qu'un bouton soit pressé
    readBtn();
    
    // Si la touche Valide est pressee, alors on passe dans le menu suivant
    if (valPressed == true){
      valPressed=false;
      switch (posMenu){
        case 0 : // auto cut-off delay
          autoCutConfig();
          menuLib[0][1]= {String(autoCutLib[autoCutVal])};
          break;
        case 1 : // //Restore Default
          restoreDefault();
          break;
      }
    }
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (bckPressed == true){
      bckPressed=false;
      keepMenu=false;
    }

    if (upPressed == true){
      upPressed=false;
      posMenu = (posMenu+1)%2;
    }
    if (dwnPressed == true){
      dwnPressed=false;
      if (posMenu == 0){posMenu =1;}else{posMenu=0;}
    }

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(menuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[posMenu][1]);
  }
}



// Fonction de paramétrage des valeurs de coupure automatique sur délai
// IN : N/A
// OUT : N/A
void autoCutConfig(){
  bool keepMenu=true;

  lcd.setCursor(0,1);
  lcd.cursor();
  lcd.blink();
    
  while(keepMenu==true){
    readBtn();
    
    if (upPressed == true){
      upPressed=false;
      autoCutVal = (autoCutVal+1)%5;
    }
    if (dwnPressed == true){
      dwnPressed=false;
      if (autoCutVal==0){autoCutVal=4;}else{autoCutVal--;}
    }
    if (valPressed == true || dwnPressed== true ){
      valPressed=false;
      dwnPressed=false;
      keepMenu=false;
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
  EEPROM.put(4, autoCutVal);
  
  lcd.noCursor();
  lcd.noBlink();
}


// Fonction de rétablissement des setup par default
// IN : N/A
// OUT : N/A
void restoreDefault(){
  bool keepMenu=true;

  while(keepMenu==true){
    lcd.setCursor(0,1);
    lcd.print(F("Confirm?"));
    readBtn();
    if (valPressed ==true ){
      valPressed =false;
      lcd.setCursor(0,1);
      lcd.print(F("...Done..."));
      
      //On stock des int, qui font donc 2 octets, donc on écrit tous les 2 octets
      //consignes
      EEPROM.put(0, 50);
      EEPROM.put(2, 50);
      
      //AutoCut config 
      EEPROM.put(4, 2);
        
      //Correction temp
      EEPROM.put(6, 0);
      EEPROM.put(8, 0);
      EEPROM.put(10, 0);
      EEPROM.put(12, 0);
      
      // Puis on recharge toutes les variables globales
      EEPROM.get(0, consigne[0]);
      EEPROM.get(2, consigne[1]);
      EEPROM.get(4, autoCutVal);
               
      delay(1500);
      keepMenu=false;
    } 
    if (bckPressed == true){
      bckPressed = false;
      keepMenu=false;
    } 

    if (upPressed == true){
      upPressed = false;
    } 
    if (dwnPressed == true){
      dwnPressed = false;
    }
  }
}

 
  
// Fonction de lecture des boutons de saisie : UP, DOWN, Valid et Back
// IN : void
// OUT : void
void readBtn(){
  byte nbBoucle=0;
  int BtnReadVal=0;
 
  // On va boucler ici tant qu'un bouton n'est pas appuyé
  // Mais on va aussi sortir toutes les 5 boucles (~0.250s)
  while (upPressed ==false && dwnPressed ==false && bckPressed ==false  && valPressed ==false && nbBoucle <5 ){
    // Lecture des boutons appuyés
    BtnReadVal = analogRead (BtnPin);
    
    // > 280 ==> Aucun bouton
    // entre 200 et 280 ==> Down
    // entre 120 et 200   => UP
    // entre 50 et 120 => Valider
    // < 50 ==> Back

    //Bouton Down
    if (BtnReadVal>=200 && BtnReadVal<280 ){
        dwnPressed =true;
        delay(200);
    }

    //Bouton Valider (inversion up/val dans la versiob HW 1.1)
    if (BtnReadVal>=120 && BtnReadVal<200){
        //valPressed =true; //v1.1
        upPressed =true; //v1.0
        delay(200);
    }

    //Bouton UP (inversion up/val dans la versiob HW 1.1)
    if (BtnReadVal>=50 && BtnReadVal<120 ){
       //upPressed = true; //v1.1
       valPressed =true; //v1.0
       delay(200); 
    }

    //Bouton Back
    if (BtnReadVal<50 ){
        bckPressed =true;
        delay(200);
    }
    nbBoucle++;
    delay(25);
  }
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

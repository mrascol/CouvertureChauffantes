//Nécessaire écriture EEPROM
#include <EEPROM.h>

//Nécessaire pour l'interruption par bouton
#include <PinChangeInterrupt.h>

// Screen Library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Version
//const String hVersion="HW=2.0    SW=3.6";
const String hVersion="HW=3.0    SW=3.6";



// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)

LiquidCrystal_I2C LCD(0x27,16,2);


#define LOGO_HEIGHT   64
#define LOGO_WIDTH    69


// Les Boutons
const byte btnDwn = 3;
const byte btnUp = 4;
const byte btnBck = 5;
const byte btnVal = 2;
bool upPressed=false;
bool dwnPressed=false;
bool bckPressed=false;
bool valPressed=false;

// Conf du Delay en secondes 
const short autoCutLst[6]={0, 600, 900, 1800, 3600, 7200};
const String autoCutLib[6]={"OFF", "10mn", "15mn", "30mn", "1h", "2h"};
byte autoCutVal = 0;

//Conf des températures
byte consigne[2][2]={{50,50},{50,50}};
short correctionTemp[4]={0,0,0,0};
float temperature[4]={0,0,0,0};
float temperaturePrev[4]={0,0,0,0};

// Variable ShortWarming
byte autoCutValShort = 1;
float minVoltShort = 9.0;

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

template< typename T, size_t N > size_t ArraySize (T (&) [N]){ return N; }


void setup() {
  LCD.init(); // initialisation de l'afficheur
  LCD.backlight();
 
  LCD.print(F("www.ae-rc.com"));
  LCD.setCursor(0,1);
  LCD.print(hVersion);

  delay (1000);

  //DEBUG
  //Serial.begin(9600);

  //Init des interruptions
  pinMode(btnUp, INPUT_PULLUP);
  pinMode(btnDwn, INPUT_PULLUP);
  pinMode(btnBck, INPUT_PULLUP);
  pinMode(btnVal, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(btnUp), btnUpFunction, FALLING);
  attachPCINT(digitalPinToPCINT(btnDwn), btnDwnFunction, FALLING);
  attachPCINT(digitalPinToPCINT(btnBck), btnBckFunction, FALLING);
  attachPCINT(digitalPinToPCINT(btnVal), btnValFunction, FALLING);

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
  // pour le contrast
  // Les corrections de températures
 
  //On stock des int, qui font donc 2 octets, donc on écrit tous les 2 octets
  EEPROM.get(0, consigne[0][0]);
  EEPROM.get(2, consigne[0][1]);
  EEPROM.get(4, autoCutVal);
  EEPROM.get(6, correctionTemp[0]);
  EEPROM.get(8, correctionTemp[1]);
  EEPROM.get(10, correctionTemp[2]);
  EEPROM.get(12, correctionTemp[3]);
  EEPROM.get(14, consigne[1][0]);
  EEPROM.get(16, consigne[1][1]);
  EEPROM.get(18, autoCutValShort);
  EEPROM.get(20, minVoltShort);  //ATTENTION C est un FLOAT sur 4 Byte. Donc la prochaine meme dispo est à 24


}



void loop() {
  const String mainMenu[3]={"1.Warming", "2.Short Warming", "3.Setup"};
  byte posMenu=0;
  //Affichage du MainMenu
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print(mainMenu[posMenu]);
  while(1){
    if (upPressed == true){
      posMenu=(posMenu+1)%3;
      upPressed=false;
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(mainMenu[posMenu]);
    }
  
    if (dwnPressed == true){
      if (posMenu == 0){
        posMenu=2;
      }
      else{
        posMenu=posMenu-1;
      }
      dwnPressed=false;
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(mainMenu[posMenu]);
    }

    if (bckPressed == true){
      bckPressed=false;
    }

    if (valPressed == true){
      valPressed=false;
      switch (posMenu){
        case 0 : warmingMenuDsp(0);break;
        case 1 : warmingMenuDsp(1);break;
        case 2 : setupMenuDsp();break;
      }
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(mainMenu[posMenu]);
    }
    delay(100);
  }
}


void setupMenuDsp(){
  const String setupMenu[6]={"1.Cut-off Delay", "2.ShortWrm Setup", "3.Calibrate", "4.Factory Reset", "5.Version", "6.Input Voltage"};
  byte posMenu=0;

  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print(setupMenu[posMenu]);
  
  while (bckPressed==false){
    //Si je suis sur la posMenu=4, alors j'affiche la version direct
    if (posMenu ==4){
      versionMenuDsp();
    }
    //Si je suis sur la posMenu=5, alors j'affiche directe la tension
    if (posMenu ==5){
      afficheTensionIn();
    }

    if (dwnPressed == true){
      if (posMenu == 0){
        posMenu=5;
      }
      else{
        posMenu=posMenu-1;
      }
      
      dwnPressed=false;
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(setupMenu[posMenu]);
      
    }
  
    if (upPressed == true){
      posMenu=(posMenu+1)%6;
      
      upPressed=false;
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(setupMenu[posMenu]);
    }

    if (valPressed == true){
      valPressed=false;
      switch (posMenu){
        case 0 : cutoffMenuDsp();break;
        case 1 : shortWarmstpMenuDsp();break;
        case 2 : calibrateMenuDsp();break;
        case 3 : factoryResetMenuDsp();break;
        case 4 : ;break;
        case 5 : ;break;
      }
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(setupMenu[posMenu]);
    }
    delay(100);
  }
  bckPressed=false;
}


 
void warmingMenuDsp(bool shortWarm){ 
  bool keepWarming=true;
  byte minutes;
  byte secondes;
  bool hideTemp=false;
  byte cycle=0;
  short autoCutValSecondes=0;
  float tensionIn;

  //On prends l'heure de démarrage
  unsigned long startWarmingTime=0;
  startWarmingTime=millis();
  
  //Si on est en ShortWarming = On surcharge la durée de CutOff à 10mn (AUTOCUTVAL = 1)
  if (shortWarm == 1){
    autoCutValSecondes = autoCutValShort*60;
  }
  else
  {
    autoCutValSecondes = autoCutLst[autoCutVal];
  }


  //On initialise l'affichage
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print(String(F("00m  FL=")) + String(consigne[shortWarm][0]) + F(" FR=")+ String(consigne[shortWarm][0]));
  LCD.setCursor(0,1);
  LCD.print(String(F("00s  RL=")) + String(consigne[shortWarm][1]) + F(" RR=")+ String(consigne[shortWarm][1]));
    
  while (keepWarming==true){
    //Mise à jour timer 
    minutes=(byte)round(((millis()-startWarmingTime)/1000/60)%99);
    secondes=(byte)round(((millis()-startWarmingTime)/1000)%60);
    
    LCD.setCursor(0,0);
    LCD.print(padding(minutes,2));
    LCD.setCursor(0,1);
    LCD.print(padding(secondes,2));
    
    //On boucle jusqu'à ce que le timer soit atteint, ou que le bouton back soit pressé
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture.
    //si la température est déconnante <-10 ou >100 On coupe tout
    switch (cycle%4){
        case 0 : //FL
            warmingCheckAdjust(sensorFL, chauffeFL, 0, consigne[shortWarm][0], 8, 0, hideTemp);
            break;
        case 1 : //FR
            warmingCheckAdjust(sensorFR, chauffeFR, 1, consigne[shortWarm][0], 14, 0, hideTemp);
            break;
        case 2 : //RL
            warmingCheckAdjust(sensorRL, chauffeRL, 2, consigne[shortWarm][1], 8, 1, hideTemp);
            break;
        case 3 : //RR
            warmingCheckAdjust(sensorRR, chauffeRR, 3, consigne[shortWarm][1], 14, 1, hideTemp);  
            break;
    }
    
    //Si touche up ou down, on passe en réglage des temp
    if (upPressed == true || dwnPressed == true){
      upPressed=false;
      dwnPressed=false;

      //On coupe la chauffe
      digitalWrite(chauffeFL, LOW);
      digitalWrite(chauffeFR, LOW);
      digitalWrite(chauffeRL, LOW);
      digitalWrite(chauffeRR, LOW);
      warmingSetup(shortWarm);
      
    }
  
    //Si touche back, on va sortir de la boucle
    if (bckPressed == true){
      bckPressed=false;
      keepWarming=false;
    }
    
    //Si touche val, on masque les temps
    if (valPressed == true){
      hideTemp=!hideTemp;
      valPressed=false;
    }

    // On Check si on a pas atteint la fin du delay
    if ((millis()-startWarmingTime)/1000 > autoCutValSecondes && autoCutVal !=0 ){
      keepWarming=false;
    }

    // On Check si on a pas atteint la tension mini paramétree
    //Mais on check aussi, si le hardware gere la tension
    tensionIn=readTensionIn();
    if (tensionIn < minVoltShort && tensionIn > 0.1){
      keepWarming=false;
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print("Voltage Too Low");
      delay(3000);
    }
    cycle=(cycle+1);
    if (cycle==252+1){
      cycle=0;
    }
    delay(5);
  }
  //On coupe la chauffe avant de sortir
  bckPressed=false;
  digitalWrite(chauffeFL, LOW);
  digitalWrite(chauffeFR, LOW);
  digitalWrite(chauffeRL, LOW);
  digitalWrite(chauffeRR, LOW);
};


// Fonction qui permet de régler les consignes de température
// IN : le port du Sensor, la consigne pour ce port, la ligne pour l'affichage, la colonne pour l'affichage
// OUT : la nouvelle température mesurée
void  warmingCheckAdjust(int sensorCurrent, byte sensorChauffe, byte rang, byte consigneCurrent, byte x, byte y, bool hideTemp){

  //lecture de la temp
  temperaturePrev[rang]=temperature[rang];
  temperature[rang]=readTemp(sensorCurrent, correctionTemp[rang]);
  
  //on COUPE si
  //Largement au dessus 
  //ou Tendance à la hausse jusqà 2° en dessous de la consigne
  //ou des temps déconnantes
  if ((temperature[rang]>=consigneCurrent + 1 ) || (temperature[rang]>consigneCurrent-1 && temperature[rang] >= temperaturePrev[rang]) || temperature[rang] < -10 || temperature[rang] > 105) {  
      digitalWrite(sensorChauffe, LOW); 
  } 

  //on CHAUFFE si
  // Largement en dessous
  // ou Tendance à la baisse jusqu'à 2° au dessus de la consigne
  if ((temperature[rang]<=consigneCurrent-1) || (temperature[rang]<consigneCurrent+1 && temperature[rang] < temperaturePrev[rang])){  

  }

  //J'affiche la temperature ou je la cache en fonction du mode
  //Si une des coordonnes d'affichage =99, c'est que j'utilise la fonction pour le calibrage
  //Je ne passe pas dans ce mode d'affichage
  if ( x != 99 && y !=99 ){
    if ( hideTemp == false && temperature[rang] > -10 && temperature[rang] < 105 && temperaturePrev[rang] > -10 && temperaturePrev[rang] < 105){
      LCD.setCursor(x, y);
      LCD.print(String((int)((temperature[rang]+temperaturePrev[rang])/2)));
    }
    else 
    {
      LCD.setCursor(x, y);
      LCD.print(F("xx"));
    }
  }
}

// Fonction qui permet de lire une température
// IN : N/A
// OUT : N/A
float readTemp(int sensorCurrent, int tempCorrectionCurrent){
  int readValue = 0;
  float resistance;
  float tempMesured;

  // Lecture de la température et conversion en °C
  readValue = analogRead(sensorCurrent);
  resistance=(float)(1023-readValue)*10000/readValue; 
  tempMesured=1/(log(resistance/10000)/B+1/298.15)-273.15+tempCorrectionCurrent;

  return tempMesured;
}

//TODO Faire une méthode, qui prend en entrée : le texte / La taille / surligne / coord
// Fonction qui permet de régler les consignes de température
// IN : N/A
// OUT : N/A
void warmingSetup(bool shortWarm){
  bool keepSetuping=1;
  byte posMenu=0;
  byte cursorPos[2]={3,13};
  
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print(F("Temp Setup:"));
  LCD.setCursor(0,1);
  LCD.print("FL=" + String(consigne[shortWarm][0]) + F("     RR=") + String(consigne[shortWarm][1]));

  LCD.setCursor(cursorPos[posMenu],1);
  LCD.cursor();
  LCD.blink();
  
  while ((keepSetuping==1)){
    //Si touche up ou down, on règle la consigne
    if (upPressed == true){
      upPressed=false;
      consigne[shortWarm][1]=consigne[shortWarm][1]+1;
      if(consigne[shortWarm][1]>95){consigne[shortWarm][1]=95;}
      LCD.setCursor(cursorPos[1],1);
      LCD.print(String(consigne[shortWarm][1]));
      LCD.setCursor(cursorPos[1],1);
      
      if (posMenu==0){
        consigne[shortWarm][0]=consigne[shortWarm][0]+1;
        if(consigne[shortWarm][0]>95){consigne[shortWarm][0]=95;}
        LCD.setCursor(cursorPos[0],1);
        LCD.print(String(consigne[shortWarm][0]));
        LCD.setCursor(cursorPos[0],1);
      }
    }

    //Si touche up ou down, on règle la consigne
    if (dwnPressed == true){
      dwnPressed=false;
      consigne[shortWarm][1]=consigne[shortWarm][1]-1;
      if(consigne[shortWarm][1]==0){consigne[shortWarm][1]=1;}
      LCD.setCursor(cursorPos[1],1);
      LCD.print(String(consigne[shortWarm][1]));
      LCD.setCursor(cursorPos[1],1);
      
      if (posMenu==0){
        consigne[shortWarm][0]=consigne[shortWarm][0]-1;
        if(consigne[shortWarm]==0){consigne[shortWarm][0]=1;}
        LCD.setCursor(cursorPos[0],1);
        LCD.print(String(consigne[shortWarm][0]));
        LCD.setCursor(cursorPos[0],1);
      }
      
    }
    //Si touche back, on va sortir de la boucle
    if (bckPressed == true){
      bckPressed=false;
      keepSetuping=0;

      LCD.noCursor();
      LCD.noBlink();
      LCD.setCursor(0,1);
      LCD.print(F("                "));
      LCD.setCursor(0,1);
      LCD.print(F("Cancel..."));
      
      delay(1000);
    }
    
    //Si touche val, on passe à la mesure suivante ou on sort si c'est la dernière
    if (valPressed == true){
      valPressed=false;
      posMenu++;
      LCD.setCursor(cursorPos[posMenu],1);
      if (posMenu==2){
        keepSetuping=0;
        
        //Avant de sortir on enregistre les consignes dans l'EEPROM
        if (shortWarm == 0){
            EEPROM.put(0, consigne[0][0]);
            EEPROM.put(2, consigne[0][1]);
        }
        else{
            EEPROM.put(14, consigne[1][0]);
            EEPROM.put(16, consigne[1][1]);
        }


        LCD.noCursor();
        LCD.noBlink();
        LCD.setCursor(0,1);
        LCD.print(F("                "));
        LCD.setCursor(0,1);
        LCD.print(F("Save..."));
        delay(500);
      }
    }
    delay(100);
  }
  //On remet la bonne trame d'affichage
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print(String(F("00m  FL=")) + String(consigne[shortWarm][0]) + F(" FR=")+ String(consigne[shortWarm][0]));
  LCD.setCursor(0,1);
  LCD.print(String(F("00s  RL=")) + String(consigne[shortWarm][1]) + F(" RR=")+ String(consigne[shortWarm][1]));
}

void shortWarmstpMenuDsp(){
  const String setupMenu[2]={"1.ShrtWrm minV", "2.ShortWrm Delay"};
  byte posMenu=0;

  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print(setupMenu[posMenu]);
  
  while (bckPressed==false){
    if (dwnPressed == true){
      if (posMenu == 0){
        posMenu=1;
      }
      else{
        posMenu=posMenu-1;
      }
      
      dwnPressed=false;
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(setupMenu[posMenu]);
      
    }
  
    if (upPressed == true){
      posMenu=(posMenu+1)%2;
      
      upPressed=false;
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(setupMenu[posMenu]);
    }

    if (valPressed == true){
      valPressed=false;
      switch (posMenu){
        case 0 : shortWarmMinVoltMenuDsp();break;
        case 1 : shortWarmDelayMenuDsp();break;
      }
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(setupMenu[posMenu]);
    }
    delay(100);
  }
  bckPressed=false;
};

// Menu pour régler la tension de coupure sur batterie
void shortWarmMinVoltMenuDsp(){
  bool keepMenu=1;
  float posMenu=autoCutVal;
  
  LCD.cursor();
  LCD.blink();
  LCD.setCursor(0,1);
  LCD.print(minVoltShort);
  LCD.setCursor(0,1);

  while (keepMenu==1){
    if (upPressed == true){
      if (minVoltShort >= 12){
        minVoltShort=12;
        
      }
      else{
        minVoltShort=minVoltShort+0.1;
      }
      upPressed=false;
      LCD.setCursor(0,1);
      LCD.print(F("                "));
      LCD.setCursor(0,1);
      LCD.print(minVoltShort);
      LCD.setCursor(0,1);
    }
  
    if (dwnPressed == true){
      if (minVoltShort <= 10){
        minVoltShort=10;
        
      }
      else{
        minVoltShort=minVoltShort-0.1;
      }
      dwnPressed=false;
      LCD.setCursor(0,1);
      LCD.print(F("                "));
      LCD.setCursor(0,1);
      LCD.print(minVoltShort);
      LCD.setCursor(0,1);
    }

    if (bckPressed == true){
      bckPressed=false;
      keepMenu=0;
      LCD.setCursor(0,1);
      LCD.print(F("Cancel...."));
      delay(1000);
    }

    if (valPressed == true){
      valPressed=false;
      autoCutVal=posMenu;
      EEPROM.put(20, minVoltShort);
      keepMenu=0;
      LCD.setCursor(0,1);
      LCD.print(F("save...."));
      delay(1000);
    }
    delay(100);
  }
  LCD.noCursor();
  LCD.noBlink();
};


// Menu pour régler le délai du ShortWarming en minutes (entre 1 et 15)
void shortWarmDelayMenuDsp(){
  bool keepMenu=1;
  byte posMenu=autoCutVal;
  
  LCD.cursor();
  LCD.blink();
  LCD.setCursor(0,1);
  LCD.print(autoCutValShort);
  LCD.setCursor(0,1);

  while (keepMenu==1){
    if (upPressed == true){
      autoCutValShort=(autoCutValShort+1)%15;
      upPressed=false;
      LCD.setCursor(0,1);
      LCD.print(F("                "));
      LCD.setCursor(0,1);
      LCD.print(autoCutValShort);
      LCD.setCursor(0,1);
    }
  
    if (dwnPressed == true){
      if (autoCutValShort == 1){
        autoCutValShort=1;
        
      }
      else{
        autoCutValShort=autoCutValShort-1;
      }
      dwnPressed=false;
      LCD.setCursor(0,1);
      LCD.print(F("                "));
      LCD.setCursor(0,1);
      LCD.print(autoCutValShort);
      LCD.setCursor(0,1);
    }

    if (bckPressed == true){
      bckPressed=false;
      keepMenu=0;
      LCD.setCursor(0,1);
      LCD.print(F("Cancel...."));
      delay(1000);
    }

    if (valPressed == true){
      valPressed=false;
      autoCutVal=posMenu;
      EEPROM.put(18, autoCutValShort);
      keepMenu=0;
      LCD.setCursor(0,1);
      LCD.print(F("save...."));
      delay(1000);
    }
    delay(100);
  }
  LCD.noCursor();
  LCD.noBlink();
};



void cutoffMenuDsp(){
  bool keepMenu=1;
  byte posMenu=autoCutVal;
  
  LCD.cursor();
  LCD.blink();
  LCD.setCursor(0,1);
  LCD.print(autoCutLib[posMenu]);
  LCD.setCursor(0,1);

  while (keepMenu==1){
    if (upPressed == true){
      posMenu=(posMenu+1)%6;
      upPressed=false;
      LCD.setCursor(0,1);
      LCD.print(F("                "));
      LCD.setCursor(0,1);
      LCD.print(autoCutLib[posMenu]);
      LCD.setCursor(0,1);
    }
  
    if (dwnPressed == true){
      if (posMenu == 0){
        posMenu=5;
        
      }
      else{
        posMenu=posMenu-1;
      }
      dwnPressed=false;
      LCD.setCursor(0,1);
      LCD.print(F("                "));
      LCD.setCursor(0,1);
      LCD.print(autoCutLib[posMenu]);
      LCD.setCursor(0,1);
    }

    if (bckPressed == true){
      bckPressed=false;
      keepMenu=0;
      LCD.setCursor(0,1);
      LCD.print(F("Cancel...."));
      delay(1000);
    }

    if (valPressed == true){
      valPressed=false;
      autoCutVal=posMenu;
      EEPROM.put(4, autoCutVal);
      keepMenu=0;
      LCD.setCursor(0,1);
      LCD.print(F("save...."));
      delay(1000);

    }
    delay(100);
  }
  LCD.noCursor();
  LCD.noBlink();
};


//Menu pour corriger les températures
void calibrateMenuDsp(){
  bool keepMenu=1;
  byte posMenu=0;
  byte tpsChauffe=0;
  unsigned long startWarmingTime=0;

  //On initialise l'affichage
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print(String(F("Mesure FL=xx.xx")));
  LCD.setCursor(0,1);
  LCD.print(String(F("Chauffe 60s")));

  //On prends l'heure de démarrage
  startWarmingTime=millis();

  while (keepMenu==1){
    switch (posMenu){
        case 0 : //FL
            warmingCheckAdjust(sensorFL, chauffeFL, 0, consigne[0][0], 99, 99, false);
            readTemp(sensorFL, correctionTemp[posMenu]);
            break;
        case 1 : //FR
            warmingCheckAdjust(sensorFR, chauffeFR, 1, consigne[0][0], 99, 99, false);
            readTemp(sensorFR, correctionTemp[posMenu]);
            break;
        case 2 : //RL
            warmingCheckAdjust(sensorRL, chauffeRL, 2, consigne[0][1], 99, 99, false);
            readTemp(sensorRL, correctionTemp[posMenu]);
            break;
        case 3 : //RR
            warmingCheckAdjust(sensorRR, chauffeRR, 3, consigne[0][1], 99, 99, false);
            readTemp(sensorRR, correctionTemp[posMenu]);
            break;
    }
     
    //On Affiche la temp Lue CORRIGEE
    LCD.setCursor(10,0);
    LCD.print(String(temperature[posMenu]));

    //Si on a fini le temps de chauffe
    //On peut agir sur la correction
    if ( tpsChauffe > 60){
        LCD.setCursor(0,1);
        LCD.print(String(F("Correction="))+paddingSigne(correctionTemp[posMenu],1));
    
        if (dwnPressed == true){
          dwnPressed=false;
          if (correctionTemp[posMenu]>-19){
              correctionTemp[posMenu]=correctionTemp[posMenu]-1;
          }
        }
  
        if (upPressed == true){
          upPressed=false;
          if (correctionTemp[posMenu]<19){
              correctionTemp[posMenu]=correctionTemp[posMenu]+1;
          }
        }
    }
    else {
      tpsChauffe=(byte)round(((millis()-startWarmingTime)/1000));
      LCD.setCursor(8,1);
      LCD.print(String(padding(tpsChauffe,2)));
      LCD.print(String(F("s")));
      delay(200);
    }

    if (bckPressed == true){
      bckPressed=false;
      
      digitalWrite(chauffeFL, LOW);
      digitalWrite(chauffeFR, LOW);
      digitalWrite(chauffeRL, LOW);
      digitalWrite(chauffeRR, LOW);     

      startWarmingTime=millis();
      tpsChauffe=0;
      
      if (posMenu==0){
        keepMenu=0;
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print(String(F("Cancel...")));
      }
      else {
        posMenu=posMenu-1;
        LCD.setCursor(7,0);
        switch (posMenu){
          case 0 : LCD.print(String(F("FL")));break;
          case 1 : LCD.print(String(F("FR")));break;
          case 2 : LCD.print(String(F("RL")));break;
          case 3 : LCD.print(String(F("RR")));break;
        }
        LCD.setCursor(0,1);
        LCD.print(String(F("Chauffe 00s    ")));
      }
    }

    if (valPressed == true){
      valPressed=false;
      
      digitalWrite(chauffeFL, LOW);
      digitalWrite(chauffeFR, LOW);
      digitalWrite(chauffeRL, LOW);
      digitalWrite(chauffeRR, LOW);     

      startWarmingTime=millis();
      tpsChauffe=0; 
      
      if (posMenu==3){
        keepMenu=0;
        //Persistence en RAM
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print(String(F("Save...")));
        EEPROM.put(6, correctionTemp[0]);
        EEPROM.put(8, correctionTemp[1]);
        EEPROM.put(10, correctionTemp[2]);
        EEPROM.put(12, correctionTemp[3]);
  
      }
      else{
        posMenu=posMenu+1;
        LCD.setCursor(7,0);
        switch (posMenu){
          case 0 : LCD.print(String(F("FL")));break;
          case 1 : LCD.print(String(F("FR")));break;
          case 2 : LCD.print(String(F("RL")));break;
          case 3 : LCD.print(String(F("RR")));break;
        }
        LCD.setCursor(0,1);
        LCD.print(String(F("Chauffe 00s    ")));        
      }
    }
    delay(50);
  }
  delay(1000);
};


void factoryResetMenuDsp(){
  bool keepMenu=1;
 
  
  LCD.setCursor(0,1);
  LCD.print(F("Confirm?"));


  while (keepMenu==1){
    if (dwnPressed == true){dwnPressed=false;}
    if (upPressed == true){upPressed=false;}

    if (bckPressed == true){
      bckPressed=false;
      keepMenu=0;
      LCD.setCursor(0,1);
      LCD.print(F("Cancel...."));
      delay(1000);
    }

    if (valPressed == true){
      valPressed=false;
      keepMenu=0;
      LCD.setCursor(0,1);
      LCD.print(F("save...."));

      //On stock des int, qui font donc 2 octets, donc on écrit tous les 2 octets
      //consignes
      EEPROM.put(0, 50);
      EEPROM.put(2, 50);
      EEPROM.put(14, 50);
      EEPROM.put(16, 50);

      //AutoCut config 
      EEPROM.put(4, 2);
      EEPROM.put(18, 10);
      EEPROM.put(20, 9.0);
      
        
      //Correction temp
      EEPROM.put(6, 0);
      EEPROM.put(8, 0);
      EEPROM.put(10, 0);
      EEPROM.put(12, 0);  

      // Puis on recharge toutes les variables globales
      EEPROM.get(0, consigne[0][0]);
      EEPROM.get(2, consigne[0][1]);
      EEPROM.get(4, autoCutVal);
      EEPROM.get(6, correctionTemp[0]);
      EEPROM.get(8, correctionTemp[1]);
      EEPROM.get(10, correctionTemp[2]);
      EEPROM.get(12, correctionTemp[3]);
      EEPROM.get(14, consigne[1][0]);
      EEPROM.get(16, consigne[1][1]);
      EEPROM.get(18, autoCutValShort);
      EEPROM.get(20, minVoltShort);

      delay(2000);
    }
    delay(100);
  }
  
  LCD.noCursor();
  LCD.noBlink();
};

void versionMenuDsp(){
  LCD.setCursor(0,1);
  LCD.print(hVersion);
};


void btnUpFunction(){
  disablePCINT(digitalPinToPCINT(btnUp));
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 100ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200) 
  {
    upPressed=true;
  }
  last_interrupt_time = interrupt_time;
  enablePCINT(digitalPinToPCINT(btnUp)); 
}

void btnDwnFunction(){
  disablePCINT(digitalPinToPCINT(btnDwn));
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 100ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200) 
  {
    dwnPressed=true;
  }
  last_interrupt_time = interrupt_time;
  enablePCINT(digitalPinToPCINT(btnDwn)); 
}


void btnBckFunction(){
  disablePCINT(digitalPinToPCINT(btnBck));
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 100ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200) 
  {
    bckPressed=true;
  }
  last_interrupt_time = interrupt_time;
  enablePCINT(digitalPinToPCINT(btnBck));
}

void btnValFunction(){
  disablePCINT(digitalPinToPCINT(btnVal));
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 100ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200) 
  {
    valPressed=true;
  }
  last_interrupt_time = interrupt_time;
  enablePCINT(digitalPinToPCINT(btnVal)); 
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
 return String(padded+number);
}

//Fonction de padding Signe des nombres
String paddingSigne( int number, byte width) {
 int currentMax=10;
 
 char signe='+';
 String padded="";

 if (number < 0){
   number=-number;
   signe='-';
 }
 else {
  signe='+';
 }
 
 for (byte i=1; i<width; i++){
   if (number < currentMax) {
     padded=padded+"0";
   }
   currentMax *= 10;
 }
 return String(signe+padded+number);
}

//Mesure la tension d'entrée
void afficheTensionIn() {
  LCD.setCursor(0,1);
  LCD.print(F("                "));
  LCD.setCursor(0,1);
  LCD.print(readTensionIn());
  LCD.print("V");
  LCD.setCursor(0,1);
}

//Mesure la tension d'entrée
float readTensionIn() {
  float U;
  long Vcc;
  short valeurA6;
  int R1=20;
  int R2=10;
  Vcc=readVcc();
  valeurA6=analogRead(A6);
  //Valeur lue sur port A6 entre 0 et 1023
  //0 = correspond à 0V
  //1023 = correspond à la tension d'alimentation du circuit = Vcc
  //On applique le pont diviseur de tension
  //On multiplie par 100 pour parler en milivot et déclarer une "simple" plutot qu'un Float
  U=valeurA6*Vcc/1000.0/1023.0*(R1+R2)/R2;
  return U;  
}

//Return la tension interne sur la board
long readVcc() {
long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
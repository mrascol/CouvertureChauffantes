//Nécessaire écriture EEPROM
#include <EEPROM.h>

//Nécessaire pour l'interruption par bouton
#include <PinChangeInterrupt.h>

// Screen Library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Version
const String hVersion="HW=2.0        SW=2.0";

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define LOGO_HEIGHT   64
#define LOGO_WIDTH    69
const unsigned char logo_bmp [] PROGMEM = {
  // 'AE-BLACK_64, 69x64px
  0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xf8, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 
  0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x38, 
  0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 
  0x01, 0xc0, 0x01, 0xff, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x03, 0x80, 0x1f, 0xff, 0xe0, 0x03, 0x80, 
  0x00, 0x00, 0x07, 0x00, 0x7f, 0x01, 0xfc, 0x01, 0x80, 0x00, 0x00, 0x0e, 0x01, 0xf0, 0x00, 0x1e, 
  0x00, 0xc0, 0x00, 0x00, 0x0c, 0x03, 0xc0, 0x00, 0x07, 0x80, 0x60, 0x00, 0x00, 0x18, 0x07, 0x00, 
  0x00, 0x01, 0xc0, 0x30, 0x00, 0x00, 0x30, 0x1e, 0x00, 0x00, 0x00, 0xe0, 0x38, 0x00, 0x00, 0x70, 
  0x18, 0x00, 0x00, 0x00, 0x70, 0x1f, 0xf8, 0x00, 0x60, 0x30, 0x00, 0x00, 0x00, 0x38, 0x3f, 0xf8, 
  0x00, 0xe0, 0x60, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x98, 0x00, 0xc0, 0xe0, 0x00, 0x00, 0x03, 0xff, 
  0xe7, 0xd8, 0x01, 0xc0, 0xc0, 0x00, 0x00, 0xff, 0xf9, 0xf8, 0x58, 0x01, 0x81, 0x80, 0x00, 0x7f, 
  0xfc, 0xfe, 0x00, 0x58, 0x01, 0x83, 0x80, 0x1f, 0xff, 0x3f, 0x00, 0x00, 0x58, 0x03, 0x03, 0x07, 
  0xff, 0xcf, 0xc0, 0x00, 0x78, 0x58, 0x03, 0x03, 0xff, 0xf3, 0xf0, 0x00, 0x1d, 0xfc, 0x58, 0x03, 
  0x7f, 0xfc, 0xfc, 0x00, 0x00, 0xfd, 0x8c, 0x58, 0x1f, 0xff, 0x3f, 0x00, 0x01, 0xe0, 0xc1, 0x8c, 
  0x58, 0xff, 0xcf, 0xc0, 0x00, 0x03, 0x80, 0xc1, 0x8c, 0x58, 0xe7, 0xe0, 0x00, 0x0f, 0xc6, 0x00, 
  0xc1, 0xf8, 0x58, 0xd8, 0x00, 0x08, 0x08, 0xcc, 0x00, 0xfd, 0xf8, 0x58, 0xd0, 0x01, 0xf8, 0x08, 
  0x48, 0x00, 0xf1, 0x8c, 0x58, 0xd0, 0x61, 0x80, 0x08, 0xd8, 0x00, 0xc1, 0x8c, 0x58, 0xd0, 0xe1, 
  0x80, 0x09, 0x98, 0x00, 0xc1, 0x8c, 0x58, 0xd0, 0xe1, 0x80, 0x0f, 0x98, 0x00, 0xc1, 0x84, 0x58, 
  0xd0, 0xb1, 0xf8, 0x48, 0xcc, 0x04, 0xc0, 0x80, 0x58, 0xd1, 0xb1, 0xf3, 0xc8, 0xce, 0x2e, 0x80, 
  0x1f, 0x98, 0xd1, 0x99, 0x80, 0x08, 0xc7, 0xe0, 0x03, 0xe0, 0x78, 0xd1, 0xf9, 0x80, 0x08, 0x60, 
  0x00, 0xfc, 0x1f, 0xf8, 0xd3, 0xf9, 0x80, 0x08, 0x00, 0x3f, 0x07, 0xff, 0x80, 0xd3, 0x0d, 0xfc, 
  0x00, 0x0f, 0xc1, 0xff, 0xf1, 0x80, 0xd2, 0x0d, 0xe0, 0x03, 0xf0, 0x3f, 0xff, 0x83, 0x00, 0xd6, 
  0x00, 0x00, 0x7c, 0x0f, 0xff, 0x03, 0x03, 0x00, 0xd4, 0x00, 0x1f, 0x03, 0xff, 0xc0, 0x03, 0x03, 
  0x00, 0xd0, 0x07, 0xe0, 0xff, 0xf0, 0x00, 0x06, 0x06, 0x00, 0xd1, 0xf8, 0x3f, 0xfe, 0x00, 0x00, 
  0x0e, 0x06, 0x00, 0xde, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x1c, 0x0e, 0x00, 0xc1, 0xff, 0xf0, 0x00, 
  0x00, 0x00, 0x18, 0x0c, 0x00, 0xff, 0xf8, 0x38, 0x00, 0x00, 0x00, 0x38, 0x18, 0x00, 0xff, 0xf0, 
  0x1c, 0x00, 0x00, 0x00, 0x70, 0x18, 0x00, 0xff, 0xf8, 0x0f, 0x00, 0x00, 0x01, 0xc0, 0x30, 0x00, 
  0x00, 0x1c, 0x07, 0x80, 0x00, 0x03, 0x80, 0x60, 0x00, 0x00, 0x0e, 0x01, 0xe0, 0x00, 0x0f, 0x00, 
  0xe0, 0x00, 0x00, 0x07, 0x00, 0x7c, 0x00, 0x7c, 0x01, 0xc0, 0x00, 0x00, 0x03, 0x80, 0x1f, 0xff, 
  0xf0, 0x03, 0x80, 0x00, 0x00, 0x01, 0xc0, 0x03, 0xff, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0xe0, 
  0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 
  0x00, 0x1e, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x03, 0xc0, 0x00, 
  0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc7, 0xfc, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

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
const short autoCutLst[4]={0, 1800, 3600, 7200};
const String autoCutLib[4]={"OFF", "30mn", "1h", "2h"};
byte autoCutVal = 0;

//Conf des températures
byte consigne[2]={50,50};
short correctionTemp[4]={0,0,0,0};
float temperature[4]={0,0,0,0};
float temperaturePrev[4]={0,0,0,0};
byte cycle;


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
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;); // Don't proceed, loop forever
  }

  //Affichage du logo de démarrage
  afficheLogo();    // Draw a small bitmap image

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
  EEPROM.get(0, consigne[0]);
  EEPROM.get(2, consigne[1]);
  EEPROM.get(4, autoCutVal);
  EEPROM.get(6, correctionTemp[0]);
  EEPROM.get(8, correctionTemp[1]);
  EEPROM.get(10, correctionTemp[2]);
  EEPROM.get(12, correctionTemp[3]);
}

void loop() {
  const String mainMenu[2]={"1.Quick Warming", "2.Setup"};
  byte posMenu=0;
  byte i=0;
  //Affichage du MainMenu
  while(1){
    display.clearDisplay(); // Clear display buffer

    for (i=0; i<ArraySize(mainMenu); i++){
    
      if (i == posMenu){
        printScreen(mainMenu[i], SSD1306_BLACK, 1, 0, 1+(i*12), display.width(), 10);   
      }
      else {
        printScreen(mainMenu[i], SSD1306_WHITE, 1, 0, 1+(i*12),display.width(), 10);
      }
    }
    display.display(); // Update screen with each newly-drawn line
    if (dwnPressed == true){
      posMenu=(posMenu+1)%2;
      dwnPressed=false;
    }
  
    if (upPressed == true){
      if (posMenu == 0){
        posMenu=1;
      }
      else{
        posMenu=posMenu-1;
      }
      upPressed=false;
    }

    if (bckPressed == true){
      bckPressed=false;
    }

    if (valPressed == true){
      valPressed=false;
      switch (posMenu){
        case 0 : warmingMenuDsp();break;
        case 1 : setupMenuDsp();break;
      }
    }
    delay(100);
  }
}


void setupMenuDsp(){
  const String setupMenu[3]={"1.Cut-off Delay", "2.Calibrate", "3.Factory Reset"};
  byte posMenu=0;
  byte i=0;
  while (bckPressed==false){
    display.clearDisplay(); // Clear display buffer
    for (i=0; i<ArraySize(setupMenu); i++){
      if (i == posMenu){
        printScreen(setupMenu[i], SSD1306_BLACK, 1, 0, 1+(i*12),display.width(), 10);
      }
      else {
        printScreen(setupMenu[i], SSD1306_WHITE, 1, 0, 1+(i*12), display.width(), 10);
      }  
    }
    display.display(); // Update screen
  
    if (dwnPressed == true){
      posMenu=(posMenu+1)%3;
      dwnPressed=false;
    }
  
    if (upPressed == true){
      if (posMenu == 0){
        posMenu=2;
      }
      else{
        posMenu=posMenu-1;
      }
      upPressed=false;
    }

    if (valPressed == true){
      valPressed=false;
      switch (posMenu){
        case 0 : cutoffMenuDsp();break;
        case 1 : calibrateMenuDsp();break;
        case 2 : factoryResetMenuDsp();break;
      }
    }
    delay(100);
  }
  bckPressed=false;
}


 
void warmingMenuDsp(){ 
  bool keepWarming=true;
  byte minutes;
  byte secondes;
  bool hideTemp=false;

  //On prends l'heure de démarrage
  unsigned long startWarmingTime=0;
  startWarmingTime=millis();
  
  //On initialise l'affichage
  drawGrid();
  cycle=0;
  while (keepWarming==true){

    //Mise à jour timer 
    minutes=(byte)round(((millis()-startWarmingTime)/1000/60)%99);
    secondes=(byte)round(((millis()-startWarmingTime)/1000)%60);
    printScreen(padding(minutes,2)+F(":")+padding(secondes,2), SSD1306_WHITE, 1, display.width()/2-14, display.height()/2-4, 1, 1);
    display.display();

    //On boucle jusqu'à ce que le timer soit atteint, ou que le bouton back soit pressé
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture.
    //si la température est déconnante <-10 ou >100 On coupe tout
    switch (cycle%4){
        case 0 : //FL
            warmingCheckAdjust(sensorFL, chauffeFL, 0, consigne[0], 20, 10, hideTemp);
            break;
        case 1 : //FR
            warmingCheckAdjust(sensorFR, chauffeFR, 1, consigne[0], display.width()/2+30, 10, hideTemp);
            break;
        case 2 : //RL
            warmingCheckAdjust(sensorRL, chauffeRL, 2, consigne[1], 20, display.height()/2+12, hideTemp);
            break;
        case 3 : //RR
            warmingCheckAdjust(sensorRR, chauffeRR, 3, consigne[1], display.width()/2+30, display.height()/2+12, hideTemp);  
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
      warmingSetup();
      drawGrid();
      
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
    if ((millis()-startWarmingTime)/1000 > autoCutLst[autoCutVal] && autoCutVal !=0 ){
      keepWarming=false;
    }
    cycle=(cycle+1);
    delay(100);
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
  if ((temperature[rang]>=consigneCurrent + 1 ) || (temperature[rang]>consigneCurrent-1 && temperature[rang] >= temperaturePrev[rang]) || temperature[rang] < -10 || temperature[rang] > 100) {  
      digitalWrite(sensorChauffe, LOW); 
      //TODO Affiche logo pas de chauffe
  } 

  //on CHAUFFE si
  // Largement en dessous
  // ou Tendance à la baisse jusqu'à 2° au dessus de la consigne
  if ((temperature[rang]<=consigneCurrent-1) || (temperature[rang]<consigneCurrent+1 && temperature[rang] < temperaturePrev[rang])){  
      digitalWrite(sensorChauffe, HIGH);
      //TODO Affiche logo de chauffe
  }


  //J'affiche la temperature ou je la cache en fonction du mode
  if ( hideTemp == false && temperature[rang] > -10 && temperature[rang] < 100){
    printScreen(String((int)((temperature[rang]+temperaturePrev[rang])/2)), SSD1306_WHITE, 1, x, y , 29, 10);
  }
  else 
  {
    printScreen(F("xx"), SSD1306_WHITE, 1, x, y , 29, 10);
  }
  display.display();
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
void warmingSetup(){
  bool keepSetuping=1;
  byte posMenu=0;
  display.clearDisplay();
  printScreen(F("Front"), SSD1306_WHITE, 1, display.width()/2-display.width()/4-2.5*8,display.height()/2-12 , 0, 0);
  printScreen(F("Rear"), SSD1306_WHITE, 1, display.width()/2+display.width()/4-2*8,display.height()/2-12 , 0, 0);

  display.setTextSize(2);
  

  while ((keepSetuping==1)){
    //On affiche les consignes
    if (posMenu==0){
      printScreen(padding(consigne[0],2), SSD1306_BLACK, 2, display.width()/2-display.width()/4-14,display.height()/2, 1, 1);
      printScreen(padding(consigne[1],2), SSD1306_WHITE, 2, display.width()/2+display.width()/4-14,display.height()/2, 1, 1);
    }
    else {
      printScreen(padding(consigne[0],2), SSD1306_WHITE, 2, display.width()/2-display.width()/4-14,display.height()/2, 1, 1);
      printScreen(padding(consigne[1],2), SSD1306_BLACK, 2, display.width()/2+display.width()/4-14,display.height()/2, 1, 1);
    }
    display.display();
    
    //Si touche up ou down, on règle la consigne
    if (upPressed == true){
      upPressed=false;
      consigne[posMenu]=consigne[posMenu]+1;
      if(consigne[posMenu]>85){consigne[posMenu]=85;}
    }

    //Si touche up ou down, on règle la consigne
    if (dwnPressed == true){
      dwnPressed=false;
      consigne[posMenu]=consigne[posMenu]-1;
      if(consigne[posMenu]==0){consigne[posMenu]=1;}
    }
    //Si touche back, on va sortir de la boucle
    if (bckPressed == true){
      bckPressed=false;
      keepSetuping=0;
      
      display.clearDisplay(); // Clear display buffer
      printScreen(F("cancel..."), SSD1306_WHITE, 1, 2, 2 , 0, 0);
      display.display();
      delay(500);
    }
    
    //Si touche val, on passe à la mesure suivante ou on sort si c'est la dernière
    if (valPressed == true){
      valPressed=false;
      posMenu++;
      if (posMenu==2){
        keepSetuping=0;
        
        //Avant de sortir on enregistre les consignes dans l'EEPROM
        EEPROM.put(0, consigne[0]);
        EEPROM.put(2, consigne[1]);
        
        display.clearDisplay(); // Clear display buffer
        printScreen(F("save..."), SSD1306_WHITE, 1, 2, 2 , 0, 0);
        display.display();
        delay(500);
      }
    }
    delay(100);
  }
}


void cutoffMenuDsp(){
  bool keepMenu=1;
  byte posMenu=autoCutVal;
  byte i=0;

  while (keepMenu==1){
    display.clearDisplay(); // Clear display buffer
    printScreen(F("Auto Cutoff delay"), SSD1306_WHITE, 1, 2, 2 , 0, 0);

    for (i=0; i<ArraySize(autoCutLib); i++){
      if (i == posMenu){
        printScreen(autoCutLib[i], SSD1306_BLACK, 1, 2,2+((i+1)*10), display.width(), 10);  
      }
      else {
        printScreen(autoCutLib[i], SSD1306_WHITE, 1, 2,2+((i+1)*10), display.width(), 10);
      }
    }
    display.display(); // Update screen
  
    if (dwnPressed == true){
      posMenu=(posMenu+1)%4;
      dwnPressed=false;
    }
  
    if (upPressed == true){
      if (posMenu == 0){
        posMenu=3;
      }
      else{
        posMenu=posMenu-1;
      }
      upPressed=false;
    }

    if (bckPressed == true){
      bckPressed=false;
      keepMenu=0;
      display.clearDisplay(); // Clear display buffer
      printScreen(F("cancel..."), SSD1306_WHITE, 1, 2, 2 , 0, 0);
      display.display();
      delay(500);
    }

    if (valPressed == true){
      valPressed=false;
      autoCutVal=posMenu;
      EEPROM.put(4, autoCutVal);
      keepMenu=0;
      display.clearDisplay(); // Clear display buffer
      printScreen(F("save..."), SSD1306_WHITE, 1, 2, 2 , 0, 0);
      display.display();
      delay(500);
    }
    delay(100);
  }
  bckPressed=false;
};


//TODO
void calibrateMenuDsp(){
};



void factoryResetMenuDsp(){
  bool keepSetuping=1;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(2,2);
  display.println(F("Factory Reset"));
  display.println(F("Are you sure?\n"));
  display.println(F("Press Val to confirm"));
  display.println(F("Press Bck to cancel"));
  display.display();

  while(keepSetuping==1){

    //Si touche up ou down, on règle la consigne
    if (upPressed == true){upPressed=false;}
    if (dwnPressed == true){dwnPressed=false;}
  
    //Si touche back, on va sortir de la boucle
    if (bckPressed == true){
      bckPressed=false;
      keepSetuping=0;
      display.clearDisplay(); // Clear display buffer
      printScreen(F("cancel..."), SSD1306_WHITE, 1, 2, 2 , 0, 0);
      display.display();
      delay(500);
    }
    
    //Si touche val, on reset
    if (valPressed == true){
      valPressed=false;
      keepSetuping=0;
        
      //On stock des int, qui font donc 2 octets, donc on écrit tous les 2 octets
      //consignes
      EEPROM.put(0, 50);
      EEPROM.put(2, 50);
      
      //AutiCut config 
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
      EEPROM.get(6, correctionTemp[0]);
      EEPROM.get(8, correctionTemp[1]);
      EEPROM.get(10, correctionTemp[2]);
      EEPROM.get(12, correctionTemp[3]);

      display.clearDisplay(); // Clear display buffer
      printScreen(F("...reset done..."), SSD1306_WHITE, 1, 2, 2 , 0, 0);
      display.display();
      delay(500);

    }
  }
};


void drawGrid() {
  display.clearDisplay(); // Clear display buffer
  //Lignes encadrement poids total
  display.drawRect(display.width()/2-20, display.height()/2-10, 40, 20, SSD1306_WHITE);

  //Lignes horizontales
  display.drawLine(0, display.height()/2, display.width()/2-20, display.height()/2, SSD1306_WHITE);
  display.drawLine(display.width()/2+20, display.height()/2, display.width()-1, display.height()/2, SSD1306_WHITE);

  //Lignes verticales
  display.drawLine(display.width()/2, 0, display.width()/2, display.height()/2-10, SSD1306_WHITE);
  display.drawLine(display.width()/2, display.height()-1, display.width()/2, display.height()/2+10, SSD1306_WHITE);

  //Affichage texte
  printScreen(F("FL"), SSD1306_WHITE, 1, 2, 2, 0, 0);
  printScreen(F("FR"), SSD1306_WHITE, 1, display.width()-13, 2, 0, 0);
  printScreen(F("RL"), SSD1306_WHITE, 1, 2,display.height()-10, 0, 0);
  printScreen(F("RR"), SSD1306_WHITE, 1, display.width()-13,display.height()- 10, 0, 0);
  
  display.display(); // Update screen with each newly-drawn line
}


void afficheLogo(void) {
  display.clearDisplay();
  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);

  printScreen(hVersion, SSD1306_WHITE, 1, 5,display.height()-10, 0, 0);

  display.display();
  delay(1000);
 
}

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

//Fonction d'affichage
//toPrint = texte à afficher
//couleur = couleur SSD1306_BLACK ou SSD1306_WHITE
//taille )= taille du texte >= 1
//x = coordonnée abscisse du texte
//y = coordonnée ordonnée du texte
//xErase/yErase
//  Si xErase =0 : on efface pas
//  Si xErase =1 : on efface en fonction de ce qu'on écrit
//  Si on sappuie sur les dimensions passee en param
void printScreen(String toPrint, unsigned int couleur, byte taille, byte x, byte y, byte xErase, byte yErase) {

  if (xErase == 1){
    if (couleur == SSD1306_WHITE){
      display.fillRect(x-taille, y-taille, toPrint.length()*6*taille+1, 10*taille, SSD1306_BLACK);
    }
    else {
      display.fillRect(x-taille, y-taille, toPrint.length()*6*taille+1, 10*taille, SSD1306_WHITE);
    }
  }

  if (xErase > 1){
    if (couleur == SSD1306_WHITE){
      display.fillRect(x-taille, y-taille, xErase, yErase, SSD1306_BLACK);
    }
    else {
      display.fillRect(x-taille, y-taille, xErase, yErase, SSD1306_WHITE);
    }
  }
  display.setCursor(x, y);
  display.setTextSize(taille);
  display.setTextColor(couleur);
  display.print(toPrint);
}  

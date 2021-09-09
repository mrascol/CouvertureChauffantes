//Nécessaire écriture EEPROM
#include <Wire.h>
#include <EEPROM.h>

//Nécessaire pour l'interruption par bouton
#include <PinChangeInterrupt.h>
const byte btnUp = 2;
const byte btnDwn = 3;
const byte btnBck = 4;
const byte btnVal = 5;

// Screen Library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Version
const String hVersion="HW=2.0";
const String sVersion="SW=2.0";

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


bool upPressed=false;
bool dwnPressed=false;
bool bckPressed=false;
bool valPressed=false;


//Menu Construction
const String mainMenu[2]={"1.Quick Warming", "2.Setup"};
const String setupMenu[3]={"1.Cut-off Delay", "2.Calibrate", "3.Factory Reset"};

// Conf du Delay en secondes 
const int autoCutLst[4]={-1, 10, 3600, 7200};
const String autoCutLib[4]={"OFF", "30min", "1h", "2h"};
byte autoCutVal = 0;

//conf des libelles
const String couvList[4]={"FL", "FR", "RL", "RR"};

//Conf des températures
int consigne[2]={50,50};
int correctionTemp[4]={0,0,0,0};
float temperature[4]={0,0,0,0};
float temperaturePrev[4]={0,0,0,0};


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

  if(autoCutVal!= 0 || autoCutVal!= 1 || autoCutVal!= 2 || autoCutVal!= 3 ){
    eepromInit();  
  }
  
}

void loop() {
  int posMenu=0;
  //Affichage du MainMenu
  while(1){
    display.clearDisplay(); // Clear display buffer

    for (int i=0; i<ArraySize(mainMenu); i++){
    
      if (i == posMenu){
        display.fillRect(0, 1+(i*12), display.width(), 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);      
      }
      else {
        display.setTextColor(SSD1306_WHITE);
      }

      display.setCursor(2,2+(i*12));
      display.print(mainMenu[i]);  
    }
    display.display(); // Update screen with each newly-drawn line
    if (upPressed == true){
      posMenu=(posMenu+1)%2;
      upPressed=false;
    }
  
    if (dwnPressed == true){
      if (posMenu-1 < 0){
        posMenu=1;
      }
      else{
        posMenu=posMenu-1;
      }
      dwnPressed=false;
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
    delay(10);
  }
}


void setupMenuDsp(){
  int posMenu=0;
  while (bckPressed==false){
    display.clearDisplay(); // Clear display buffer
    for (int i=0; i<ArraySize(setupMenu); i++){
      if (i == posMenu){
        display.fillRect(0, 1+(i*12), display.width(), 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);      
      }
      else {
        display.setTextColor(SSD1306_WHITE);
      }
    
      display.setCursor(2,2+(i*12));
      display.print(setupMenu[i]);  
    }
    display.display(); // Update screen
  
    if (upPressed == true){
      posMenu=(posMenu+1)%3;
      upPressed=false;
    }
  
    if (dwnPressed == true){
      if (posMenu-1 < 0){
        posMenu=2;
      }
      else{
        posMenu=posMenu-1;
      }
      dwnPressed=false;
    }

    if (valPressed == true){
      valPressed=false;
      switch (posMenu){
        case 0 : cutoffMenuDsp();break;
        case 1 : calibrateMenuDsp();break;
        case 2 : factoryResetMenuDsp();break;
      }
    }
    delay(10);
  }
  bckPressed=false;
}


 
void warmingMenuDsp(){ 
  bool keepWarming=1;
  byte minutes;
  byte secondes;
  byte cycle;
  byte hideTemp=0;

  //On prends l'heure de démarrage
  unsigned long startWarmingTime=0;
  startWarmingTime=millis();
  
  //On initialise l'affichage
  drawGrid();
  cycle=0;
  while (keepWarming==1){

    //On boucle jusqu'à ce que le timer soit pressé, ou que le bouton back soit pressé
    // On check la température et on ajuste la tension qu'on pousse sur chaque couverture.
    switch (cycle){
        case 0 : //FL
            temperaturePrev[0]=temperature[0];
            temperature[0]=warmingCheckAdjust(sensorFL, chauffeFL, temperaturePrev[0], consigne[0], correctionTemp[0], 20, 10, hideTemp);
            break;
        case 1 : //FR
            temperaturePrev[1]=temperature[1];
            temperature[1]=warmingCheckAdjust(sensorFR, chauffeFR, temperaturePrev[1], consigne[0], correctionTemp[1], display.width()/2+20, 10, hideTemp);
            break;
        case 2 : //RL
            temperaturePrev[2]=temperature[2];
            temperature[2]=warmingCheckAdjust(sensorRL, chauffeRL, temperaturePrev[2], consigne[1], correctionTemp[2], 20, display.height()/2+12, hideTemp);
            break;
        case 3 : //RR
            temperaturePrev[3]=temperature[3];
            temperature[3]=warmingCheckAdjust(sensorRR, chauffeRR, temperaturePrev[3], consigne[1], correctionTemp[3], display.width()/2+20, display.height()/2+12, hideTemp);
            break;
    }
    
    //Mise à jour timer 
    minutes=round(((millis()-startWarmingTime)/1000/60)%99);
    secondes=round(((millis()-startWarmingTime)/1000)%60);
    display.fillRect(display.width()/2-14, display.height()/2-4, 29, 8, SSD1306_BLACK);
    display.setCursor(display.width()/2-14, display.height()/2-4);
    display.print(String(padding(minutes,2))+F(":")+String(padding(secondes,2)));
    display.display();
    
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
      keepWarming=0;
    }
    
    //Si touche val, on masque les temps
    if (valPressed == true){
      valPressed=false;
      hideTemp=(hideTemp+1)%2;
    }


    // On Check si on a pas atteint la fin du delay
    if ((millis()-startWarmingTime)/1000 > autoCutLst[autoCutVal]){
      keepWarming=0;
    }
    cycle=(cycle+1)%4;
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
int warmingCheckAdjust(int sensorCurrent, int sensorChauffe, float prevTemp, int consigneCurrent, int tempCorrectionCurrent, int x, int y, byte hideTemp){
  int readValue = 0;
  float resistance;
  float tempMesured;

  
  // Lecture de la température et conversion en °C
  readValue = analogRead(sensorCurrent);
  resistance=(float)(1023-readValue)*10000/readValue; 
  tempMesured=1/(log(resistance/10000)/B+1/298.15)-273.15+tempCorrectionCurrent;

  //On check si on doit couper la chauffe

  //Largement au dessus ou Tendance à la hausse à consigne et 2° pres au dessus
  if ((tempMesured>consigneCurrent + 2 ) || (tempMesured>=consigneCurrent-2 && tempMesured > prevTemp)) {  
      digitalWrite(sensorChauffe, LOW); 
      //TODO Affiche logo pas de chauffe
  } 

  // Largement en dessous
  if (tempMesured<consigneCurrent-1){  
      digitalWrite(sensorChauffe, HIGH);
      //TODO Affiche logo de chauffe
  }

  //J'affiche la temperature ou je le cache en fonction du mode
  display.fillRect(x,y, 29, 8, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x,y);
  if ( hideTemp == 0 ){
    tempMesured=0;
    display.print((String)tempMesured + F("°C"));  
  }
  else 
  {
    display.print(F("xxxx"));
  }
  return tempMesured;
}


//TODO Faire une méthode, qui prend en entrée : le texte / La taille / surligne / coord
// Fonction qui permet de régler les consignes de température
// IN : N/A
// OUT : N/A
void warmingSetup(){
  
  bool keepSetuping=1;
  int posMenu=0;
  display.clearDisplay();
  display.setCursor(display.width()/2-display.width()/4-2.5*8,display.height()/2-12);
  display.print(F("Front"));
  display.setCursor(display.width()/2+display.width()/4-2*8,display.height()/2-12);
  display.print(F("Rear"));
  display.setTextSize(2);
  

  while ((keepSetuping==1)){
    //On affiche les consignes
    if (posMenu==0){
      display.fillRect(display.width()/2-display.width()/4-15,display.height()/2-1, 24, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
      display.setCursor(display.width()/2-display.width()/4-14,display.height()/2);
      display.print(String(padding(consigne[0],2)));

      display.fillRect(display.width()/2+display.width()/4-15,display.height()/2-1, 24, 16, SSD1306_BLACK);
      display.setTextColor(SSD1306_WHITE); 
      display.setCursor(display.width()/2+display.width()/4-14,display.height()/2);
      display.print(String(padding(consigne[1],2)));

    }
    else {
      display.fillRect(display.width()/2-display.width()/4-15,display.height()/2-1, 24, 16, SSD1306_BLACK);
      display.setTextColor(SSD1306_WHITE); 
      display.setCursor(display.width()/2-display.width()/4-14,display.height()/2);
      display.print(String(padding(consigne[0],2)));

      display.fillRect(display.width()/2+display.width()/4-15,display.height()/2-1, 24, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK); 
      display.setCursor(display.width()/2+display.width()/4-14,display.height()/2);
      display.print(String(padding(consigne[1],2)));
    }
    display.display();
    
    //Si touche up ou down, on règle la consigne
    if (upPressed == true){
      upPressed=false;
      consigne[posMenu]=consigne[posMenu]+1;
      if(consigne[posMenu]>75){consigne[posMenu]=75;}
            
    }

    //Si touche up ou down, on règle la consigne
    if (dwnPressed == true){
      dwnPressed=false;
      consigne[posMenu]=consigne[posMenu]-1;
      if(consigne[posMenu]<0){consigne[posMenu]=0;}

    }
    //Si touche back, on va sortir de la boucle
    if (bckPressed == true){
      bckPressed=false;
      keepSetuping=0;
      display.clearDisplay(); // Clear display buffer
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(2,2);
      display.print(F("cancel..."));
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
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(2,2);
        display.print(F("save..."));
        display.display();
        delay(500);
      }
    }
  }
}



void cutoffMenuDsp(){
  bool keepMenu=1;
  int posMenu=autoCutVal;

  while (keepMenu==1){
    display.clearDisplay(); // Clear display buffer
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(2,2);
    display.print(F("Auto Cutoff delay"));
    for (int i=0; i<ArraySize(autoCutLib); i++){
      if (i == posMenu){
        display.fillRect(0, 1+((i+1)*10), display.width(), 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);      
      }
      else {
        display.setTextColor(SSD1306_WHITE);
      }
    
      display.setCursor(2,2+((i+1)*10));
      display.print(autoCutLib[i]);  
    }
    display.display(); // Update screen
  
    if (upPressed == true){
      posMenu=(posMenu+1)%4;
      upPressed=false;
    }
  
    if (dwnPressed == true){
      if (posMenu-1 < 0){
        posMenu=3;
      }
      else{
        posMenu=posMenu-1;
      }
      dwnPressed=false;
    }

    if (bckPressed == true){
      bckPressed=false;
      keepMenu=0;
      display.clearDisplay(); // Clear display buffer
      display.setCursor(2,2);
      display.print(F("cancel..."));
      display.display();
      delay(500);
    }

    if (valPressed == true){
      valPressed=false;
      autoCutVal=posMenu;
      EEPROM.put(4, autoCutVal);
      keepMenu=0;
      display.clearDisplay(); // Clear display buffer
      display.setCursor(2,2);
      display.print(F("save..."));
      display.display();
      delay(500);
    }
    delay(10);
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
    }
    
    //Si touche val, on reset
    if (valPressed == true){
      valPressed=false;
      keepSetuping=0;
        
      eepromInit();

    }
  }
};

void eepromInit(){
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

      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(2,2);
      display.println(F("...reset done..."));
      display.display();

      delay(1500);
}

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
  display.setCursor(2,2);
  display.print(F("FL"));  

  display.setCursor(display.width()-13,2);
  display.print(F("FR"));  

  display.setCursor(2,display.height()-10);
  display.print(F("RL"));  

  display.setCursor(display.width()-13,display.height()- 10 );
  display.print(F("RR"));  
  
  display.display(); // Update screen with each newly-drawn line
}


void afficheLogo(void) {
  display.clearDisplay();
  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);

  
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setTextSize(0.5);             // Draw 2X-scale text
  display.setCursor(5,display.height()-10 );
  display.print(hVersion);  
  display.setCursor(display.width()-40 ,display.height()-10 );
  display.print(sVersion);   
  display.display();
  delay(3000);
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
 return padded+number;
}

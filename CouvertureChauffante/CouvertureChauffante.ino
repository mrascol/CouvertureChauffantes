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

// include the library code:
#include <LiquidCrystal.h>
// Debug MODE
bool dbgMode = 1;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize Bouton
int upBtn = 7;
int upBtnVal = 0;
int downBtn = 8;
int downBtnVal = 0;
int yesBtn = 10;
int yesBtnVal = 0;
int backBtn = 9;
int backBtnVal = 0;

// Conf du contrast
int screenContrast=6;
int screenContrastLst[5]={200, 150, 100,50, 10};
String screenContrastLib[5]={"=", "====", "========", "============", "================"};
int screenContrastVal = 2;

// Conf du Delay en secondes 
int autoCutLst[5]={-1, 10, 3600, 7200, 14400};
String autoCutLib[5]={"OFF", "30min", "1h", "2h", "4h"};
int autoCutVal = 3;

//Conf tes températures
int consigne[4]={50,50,50,50};
int sensorFL = A0;
int sensorFLVal=0;
int rawValue = 0;

void setup() {

  // Init Serial
  if (dbgMode =1 ){
    Serial.begin(9600);
    while(!Serial);
  }
  // Btn initialize
  pinMode(upBtn, INPUT_PULLUP);
  pinMode(downBtn, INPUT_PULLUP);
  pinMode(yesBtn, INPUT_PULLUP);
  pinMode(backBtn, INPUT_PULLUP);
  pinMode(screenContrast, OUTPUT);
  pinMode(sensorFL, INPUT);


  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  analogWrite(screenContrast, screenContrastLst[screenContrastVal]);
  
  // Print a welcome message to the LCD.
  lcd.print("www.ae-rc.com");
  delay (2000);

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
    posMenuNew = readBtn(posMenu, 3);
  
    if (dbgMode==1){Serial.println(fctName+"|posMenuNew="+String(posMenuNew));}
    
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

  while ((keepWarming==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 1);
  
    if (dbgMode==1){Serial.println(fctName+"|posMenuNew="+String(posMenuNew));}
    
    // Si la touche Valide est pressee, alors on passe dans l'écran de réglage des temps
    if (posMenuNew == -1){
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
    if (dbgMode==1){Serial.println(fctName+"|startWarmingTime="+String(startWarmingTime)+"|millis="+String(millis()));}
    if ((millis()-startWarmingTime)/1000 > autoCutLst[autoCutVal]){
      keepWarming=0;
    }

    // On check la température et on ajuste la tension qu'on pousse sur chaque cuverture
    // TODO - voir s'il ne faut pas réduire le timer à 5s dans le fonction ReadBtn
    //1. read les 4 temps
    //2. ajuster la tension
    //3. Ajuster l'affichage
    //--> A mettre dans une fonction dédié, au moins les steps 1 et 2
    rawValue = analogRead(sensorFL);
    sensorFLVal = 5 * rawValue * 100 / 1024;
    if (dbgMode==1){Serial.println(fctName+"|FL="+String(sensorFLVal));}
  
    posMenu=posMenuNew;
    lcd.clear();
    lcd.noCursor();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print(menuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[posMenu][1]);
     
  }
}


void warmingSetup(){
  String fctName="warmingSetupMenu";

  int posMenu=0;
  int posMenuNew=0;
  
  int cursorPos[4]={4,13, 4, 13};
  
  int cursorPosCurrent=0;
  int cursorLine=0;
  bool keepSetuping=1;
  
  //Menu Construction
  String menuLib[1][2];
  menuLib[0][0]= {"FL="+String(consigne[0])+"    FR="+String(consigne[1])};
  menuLib[0][1]= {"RL="+String(consigne[2])+"    RR="+String(consigne[3])};

  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  //On commence par le FL
  //On détermine la ligne de l'afficheur
  if (cursorPosCurrent>1){
    cursorLine=1;
  }
  else { 
    cursorLine=0;
  }
  //On positionne curseur on bon endroit et on le fait clignoter
  lcd.setCursor(cursorPos[cursorPosCurrent],cursorLine);
  lcd.cursor();
  lcd.blink();


  while ((keepSetuping==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(consigne[cursorPosCurrent], 75);
  
    if (dbgMode==1){Serial.println(fctName+"|posMenuNew="+String(posMenuNew));}
    
    // Si la touche Valide est pressee, alors on passe au régalge suivant
    if (posMenuNew == -1){
      
      if (cursorPosCurrent==3){
        cursorPosCurrent=0;
      }
      else{
        cursorPosCurrent++;
      }
      posMenuNew=consigne[cursorPosCurrent];
    }
    
    // si la touche back est pressee, alors on revient à l'écran d'avant
    if (posMenuNew==-2){
      posMenuNew=consigne[cursorPosCurrent];
      keepSetuping=0;
    }

    //On détermine la ligne de l'afficheur
    if (cursorPosCurrent>1){
      cursorLine=1;
    }
    else
    { 
      cursorLine=0;
    }

    // On met à jour la consigne
    consigne[cursorPosCurrent]= posMenuNew;
    // Si on est sur la premiere colonne, on met à jour aussi la rouge Droite
    
    if ((cursorPosCurrent==0)||(cursorPosCurrent==2)){
      consigne[cursorPosCurrent+1]= posMenuNew;
    }

    // On met à jour le texte et on affiche
    menuLib[0][0]= {"FL="+String(consigne[0])+"    FR="+String(consigne[1])};
    menuLib[0][1]= {"RL="+String(consigne[2])+"    RR="+String(consigne[3])};

    //TODO - sécurité
    // Pendant qu'on est dans cette fonction... on ne check pas la temps et donc on ne régul pas la tension
    // Ajouter une sécurité, si aucun bouton n'est touché pendant 1min... alors forcer le retour.
    
    lcd.clear();
    lcd.noCursor();
    lcd.noBlink();
    lcd.setCursor(0,0);
    lcd.print(menuLib[posMenu][0]);
    lcd.setCursor(0,1);
    lcd.print(menuLib[posMenu][1]);
 
    //On positionne curseur on bon endroit et on le fait clignoter
    lcd.setCursor(cursorPos[cursorPosCurrent],cursorLine);
    lcd.cursor();
    lcd.blink();
 
    
  }
  lcd.noCursor();
  lcd.noBlink();
}
void setupMenu(){
  String fctName="setupMenu";

  int posMenu=0;
  int posMenuNew=0;
  bool keepMenu=1;
  //Menu Construction
  String menuLib[4][2];
  menuLib[0][0]= {"1.Default Temps"};
  menuLib[0][1]= {""};
  menuLib[1][0]= {"2.Contrast"};
  menuLib[1][1]= {String(screenContrastLib[screenContrastVal])};
  menuLib[2][0]= {"3.Buzz Power"};
  menuLib[2][1]= {"TODO"};
  menuLib[3][0]= {"4.Auto cut-off Delay"};
  menuLib[3][1]= {String(autoCutLib[autoCutVal])};
  
  //On affiche le premier Menu
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menuLib[posMenu][0]);
  lcd.setCursor(0,1);
  lcd.print(menuLib[posMenu][1]);

  while ((keepMenu==1)){
    // On attend qu'un bouton soit pressé
    posMenuNew = readBtn(posMenu, 4);
  
    if (dbgMode==1){Serial.println(fctName+"|posMenuNew="+String(posMenuNew));}
    
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

void contrastConfig(){
  String fctName="contrastConfig";
    
  int posMenuNew=0;
  bool keepMenu=1;

  lcd.setCursor(0,1);
  lcd.cursor();
  lcd.blink();
    
  while(keepMenu==1){
    posMenuNew = readBtn(screenContrastVal, 5);
    if (posMenuNew ==-1 || posMenuNew==-2){
      posMenuNew=screenContrastVal;
      keepMenu=0;
    }
    else{
      screenContrastVal=posMenuNew;
    }
  
    analogWrite(screenContrast, screenContrastLst[screenContrastVal]);
  
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print(String(screenContrastLib[screenContrastVal]));
    lcd.setCursor(0,1);
    lcd.cursor();
    lcd.blink();    
  }
  lcd.noCursor();
  lcd.noBlink();    
}


void autoCutConfig(){
  String fctName="autoCutConfig";
    
  int posMenuNew=0;
  bool keepMenu=1;

  lcd.setCursor(0,1);
  lcd.cursor();
  lcd.blink();
    
  while(keepMenu==1){
    posMenuNew = readBtn(autoCutVal, 5);
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

  lcd.noCursor();
  lcd.noBlink();
}

 
// Fonction de lecture des boutons de saisie : UP, DOWN, Valid et Back
// IN : Id actuel du Menu, Nb max d'éléments dans ce menu
// OUT : Nouvel ID de Menu 
//prends la valeur -1 c'est le bouton valider
//prends la valeur -2 c'est le bouton Back

int readBtn(int maPosMenu, int maxNbElts){
  String fctName="readBtn";
  bool btnPressed=0;
  int nbBoucle=0;
 
  // On va boucler ici tant qu'un bouton n'est pas appuyé
  // Mais on va aussi sortir toutes les 100 boucles (~10s)
  while (btnPressed ==0 && nbBoucle <100 ){
    // Lecture des boutons appuyés
    upBtnVal = digitalRead (upBtn);
    downBtnVal = digitalRead (downBtn);
    backBtnVal = digitalRead (backBtn);
    yesBtnVal = digitalRead (yesBtn);
    
    // 1 = bouton non pressé / 0 = bouton pressé
   // if (dbgMode==1){Serial.println(fctName+"|upBtnVal="+String(upBtnVal)+"|downBtnVal="+String(downBtnVal)+"|backBtnVal="+String(backBtnVal)+"|yesBtnVal="+String(yesBtnVal));}
  
    if (upBtnVal == 0 ){
       btnPressed=1;
       if (maPosMenu == maxNbElts-1){
          maPosMenu=0;
       }
       else {
          maPosMenu++;
       }
       delay(300); 
    }
    if (downBtnVal == 0 ){
        btnPressed=1;
        if (maPosMenu == 0){
            maPosMenu=maxNbElts-1;
        }
        else {
            maPosMenu--;
        } 
        delay(300);
    }
    if (yesBtnVal == 0 ){
        btnPressed=1;
        maPosMenu = -1;
        delay(300);
    }  
    if (backBtnVal == 0 ){
        btnPressed=1;
        maPosMenu = -2;
        delay(300);
    }
    nbBoucle++;
    delay(100);
  }
  return maPosMenu;
}

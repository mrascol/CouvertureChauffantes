#include <string.h>
#include <stdlib.h>

//Prends la valeur du menu courant
char  **currentMenu =NULL;

//Localication à tout moment : quel menu et quel position dans le menu
char fctName[20];


void setup() {
  strcpy(fctName, "setup");
  Serial.println(fctName);
  
  // Init Serial
  Serial.begin(9600);
  delay(500);
}

void loop() {
  strcpy(fctName, "mainMenu");
  Serial.println(fctName);
  Serial.println("toto");

  char  **mainMenuLib =NULL;
  mainMenuLib = (char **) malloc( 2 * sizeof (char*));
  mainMenuLib[0] = (char *) malloc( 20 * sizeof(char) );
  mainMenuLib[1] = (char *) malloc( 20 * sizeof(char) );
  mainMenuLib[2] = (char *) malloc( 20 * sizeof(char) );
  strcpy(mainMenuLib[0], "1.Quick Warning");
  strcpy(mainMenuLib[1], "2.Setup");
  strcpy(mainMenuLib[2], "END");

  currentMenu=mainMenuLib;
  int nbLineMenu=lineCountMenu();
  Serial.println(nbLineMenu);

 


  while ((1)){
    delay(2000);
    Serial.println(fctName);
  }
}

int lineCountMenu(void){
// J'ai paramétré tous mes tableaux pour avoir le mot "END" en fin à chaque fois

  int nbElt=0;
  bool found=false;
  while (!found){
    Serial.println(currentMenu[nbElt]);
    if (strcmp(currentMenu[nbElt],"END") ==0){
      found=true;
    }
    else{
      nbElt++;
    }
  }
  return nbElt;
}

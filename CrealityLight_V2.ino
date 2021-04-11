/* V3: utilisation d'une entrée GPIOSHUTDOWN

 * FONCTIONNEMENT LED : 
 * si le pi n'a pas activé la lumière LED2  alors l'utilisateur peut allumer éteindre avec respectivement un appui simple et un double appui
 * si le pi n'a pas activé la lumière LED1, un appui long allume LED1 et le relachement l'éteint
 * si l'utilisateur appui 3 fois, éteint/allume la LED de l'extrudeur * Fonctionnement Relais/extinction
 * la sortie LED3 n'est pas utilisé
 * FONCTIONNEMENT EXTINCTION
 * si l'utilisateur appui 4 fois, l'Arduino éteindra le PI lorsque le PI éteindra l'imprimante (mode suicide). Autre 4 click = annule l'extinction
 *   si l'utilisateur a validé l'extinction du PI par le bouton, alors éteindra le PI en activant la pin "RPIReset" après un certain délais (idéalement passage de 0 à 1 pui de 1 à 0)
 *   Mais l'utilisateur peu désactiver cette extinction avec la même séquence (4 appui)
 * si l'utilisateur appui 5 fois, l'Arduino éteind le PI immédiatement et éteindra l'imprimante après un délais pour la laisser refroidir. Pas d'annulation possible
 * Si l'utilisateur fait un appui long sur le bouton, alors l'Arduino éteint tout immédiatement (imprimante et PI)
 * NOTE : a la sélection de 4 ou 5 appuis, la LED de l'extruder clignotera du meme nombre de click, pour valider la prise en compte
 * A FAIRE : 
 *    Alimentation de l'imprimante au démarrage si on appuie sur le bouton au démarage
 */

/*        PINOUT
 *        ------
 *          VDD  |      |GND
 *RelayIN/0/PA4  |      |PA3/10 /RelayOut
 *LED2IN /1/PA5  |      |PA2/9  /LedPowerOut3
 *LED1IN /2/PA6  |      |PA1/8  /LedPowerOut2
 *RPIRst /3/PA7  |      |PA0/11 /ISP
 *PushBut/4/PB3  |      |PB0/7  /LedPowerOut1
 *OctoRdy/5/PB2  |      |PB1/6  /LedOut
 *        ------*/

/* SOIC  Pin   Arduino  ADC0  ADC1  DAC0  USART0  SPI0  TWI0  TCA0  TCBn    TCD0  CCL   
 * 10    PA0   11       AIN0  -     -     -       -     -     -     -       -     LUT0-IN0  > ISP
 * 11    PA1   8        AIN1  -     -     TXDbis  MOSI  SDA   -     -       -     LUT0-IN1  > LedPowerOut2
 * 12    PA2   9        AIN2  -     -     RXDbis  MISO  SCL   -     -       -     LUT0-IN2  > LedPowerOut3
 * 13    PA3   10       AIN3  -     -     XCKbis  SCK   -     WO3   TCB1 WO -     -         > RelayOut
 * 14    GND            -     -     -     -       -     -     -     -       -     -         >
 * 1     VDD            -     -     -     -       -     -     -     -       -     -         >
 * 2     PA4   0        AIN4  AIN0  -     XDIRbis SS          WO4           WOA   LUT0-OUT  > RelayIN > RPI PIN 33
 * 3     PA5   1        AIN5  AIN1  -     -       -     -     WO5   TCB0 WO WOB             > LED2IN > RPI PIN 32
 * 4     PA6   2        AIN6  AIN2  OUT                                                     > LED1IN > RPI PIN 31
 * 5     PA7   3        AIN7  AIN3  -     -       -     -     -     -       -     LUT1-OUT  > RPIReset
 * 6     PB3   4        -     -     -     RXD     -     -     WO0bis                        > PshBut
 * 7     PB2   5        -     -     -     TXD     -     -     WO2                           > OctoReady > > RPI PIN 38 (BCM GPIO20)
 * 8     PB1   6        AIN10 -     -     XCK     -     SDA   WO1                           > LedOut
 * 9     PB0   7        AIN11 -     -     XDIR    -     SCL   WO0                           > LedPowerOut1
 */

#include "EButton.h"
#define DEBUGMODE         // le laisser car le reste est pas testé (prise en main )

/* ---------------------------------------------------------- */
/* ------------------  HARDWARE  ---------------------------- */
/* ---------------------------------------------------------- */
/* Ajouter ATTINY1614 dans Arduino : 
 *  https://create.arduino.cc/projecthub/john-bradnam/using-the-new-attiny-processors-with-arduino-ide-612185
 * Arduino : MegaTinyCore>>ATTINY1614....
 * Bootloader : jtag2updi (megaTinyCore) à base d'un Arduino Nano Old bootloader
 * Info > BOD Volt:2,6V // Clock 1MHz // Mili by default // UART close to 3V
 * please reade this great tutorial to know what you can do with ATTINY MegaTinyCore : https://github.com/SpenceKonde/megaTinyCore 
*/
/* ---------------------------------------------------------- *
 * -------------------  PINOUT  ----------------------------- *
 * ---------------------------------------------------------- */
#define LedPowerOut1        7  //PB0 > SORTIE transistor LED
#define LedPowerOut2        8  //PA1 > SORTIE transistor LED
#define LedPowerOut3        9  //PA2 > SORTIE transistor LED
#define LedOut              6  //PB1 > SORTIE simple LED pour extrudeur
#define RelayOut            10 //PA3 > SORTIE relais 24V/16A
#define RPIReset            3  //PA7 > SORTIE : sortie à la 0 pour éteinde le pi
#define PshBut              4  //PB3 > ENTREE SEULE bonton poussoir en pull-up de commande pour interface homme Arduino
#define LED1IN              2  //PA6 > ENTREE VENANT DU PI
#define LED2IN              1  //PA5 > ENTREE VENANT DU PI
#define RelayIN             0  //PA4 > ENTREE VENANT DU PI
#define GPIOSHUTDOWNIN      5  //PB2 > ENTREE VENANT DU PI INDIQUANT SI OCTOPRINT EST PRET OU PAS

EButton button(PshBut);     //bouton pressé quand à la masse

/* ---------------------------------------------------------- */
/* -----------------  CONSTANTES  --------------------------- */
/* ---------------------------------------------------------- */
// pour séquence d'initialisation
#define DelaisStartup                         60000    // délais avant que l'ATTINY commence sa boucle principale et allume le relais et les lumières. nécessaire car pendant le boot, le PI allume le relais temporairement

// variables 
#define DureePression                         984    // en milli secondes
#define DureePressionLongue                   3000   // en milli secondes
#define DelaisExtinctionPiParPi               1000   // délais (ms) entre le moment ou l'imprimante est éteinte par le PI et le moment ou le PI est éteint par l'Arduino
#define DelaisExtinctionPiRefroidissement    120000  // délais (ms) entre le moment ou l'Arduino détecte une extinction et le moment ou il éteint l'imprimante (délais nécessaire pour refroidir le bed et l'extrudeur)
#define DelaisExtinctionPiImmediat            1000   // délais (ms) entre le moment ou l'imprimante est éteinte et le moment ou le PI est éteint (extinction immédiate)
#define DelaisReboot                          60000   // délais (ms) entre le moment ou l'imprimante est éteinte et le moment ou le PI est éteint (extinction immédiate)
#define DeglitchRelaisIn                      400    // délais (ms) pendant lequel l'entrée venant du pi doit rester à 0 avant l'extinction réelle du relais de l'imprimante

#define LED1PWM                      255          // valeur de PWM entre 0 et 255
#define LED2PWM                      255
#define LED3PWM                      255          // pas implémenté pour le moment
#define LED4PWM                      255          // sortie extrudeur

#define PIRESETE                      1           // Etat a applique pour éteindre le PI
#define PIFONCTIONNE                  0           // Etat a applique pour laisser le PI fonctionner

// constantes pour mieux lire l'état de GPIOSHUTDOWNSequenceVerif
#define STARTUP                       0
#define WAITTOBOOT                    1
#define BOOTED                        2
#define POTENTIALREBOOTED             3
#define TURNEDOFF                     4

/* pour changer la vitesse de clock, pas utilisé
#define MCLK_PRESC_2    0x00
#define MCLK_PRESC_4    0x01
#define MCLK_PRESC_8    0x02
#define MCLK_PRESC_16   0x03
#define MCLK_PRESC_32   0x04
#define MCLK_PRESC_64   0x05
#define MCLK_PRESC_6    0x08h
#define MCLK_PRESC_10   0x09
#define MCLK_PRESC_12   0x0A
#define MCLK_PRESC_24   0x0B
#define MCLK_PRESC_48   0x0C */

/* ---------------------------------------------------------- */
/* -----------------  VARIABLES  ---------------------------- */
/* ---------------------------------------------------------- */
bool ExtinctionPiParPi = false;      // TRUE, si l'utilisateur a demandé à éteindre le PI en même temps que l'imprimante, 
bool ExtinctionPiRefroidissement = false;      // TRUE, si l'utilisateur a demandé à éteindre le PI après un délais de refroidissement, 
bool ExtinctionPiImmediat = false;      // TRUE, si l'utilisateur a demandé à éteindre le PI immédiatement 
bool EntreePiRelais = false;            //variables pour le deglich de l'entrée relais venant du PI
bool EntreePiRelaisPrev = false;
bool EtatRelais = false;                // donne l'état du relais

uint8_t EtatRelaisBoot = 0;                // donne l'état du relais au boot du PI
uint8_t EtatGPIOSHUTDOWNIn = 0;            // donne l'état de la pin de status "GPIOShutdown" au boot, pour voir si elle bagotte ou reste à 0
uint8_t GPIOSHUTDOWNSequenceVerif = STARTUP;     // donne l'état du relais au boot du PI
bool EtatBoutonBoot = false;                // donne l'état du bouton poussoir au boot
uint8_t EtatBoutonBootPrev = 0;             // donne l'état du relais au boot du PI, UINT pour le comparer directement à la lecture de l'entrée qui renvoie un entier 0 ou 1 (pourrait fonctionner avec un bool mais bon....)
bool BoutonNotActivated = true;             // indique si le bouton a été activé ou non pendant le boot
bool BootTimerEnded = false;                // indique si le timer de boot s'est terminé sans changement d'état de pin du PI

bool EntreePiLED1 = false;            //variables pour le deglich de l'entrée LED1 venant du PI
bool EntreePiLED1Prev = false;
bool EntreePiLED2 = false;            //variables pour le deglich de l'entrée LED2 venant du PI
bool EntreePiLED2Prev = false;

bool PiEteint = false;          // TRUE, si le pi a été éteint par l'Arduino 
unsigned long TimeExtinctionPiParPi=0;
unsigned long TimeExtinctionPiRefroidissement=0;
unsigned long TimeExtinctionPiImmediat=0;
unsigned long TimePotentialReboot=0;

unsigned long TimeDeglichRelais=0;
unsigned long TimeRelaisOff=0;

uint8_t PrevLED1IN = 0;
uint8_t PrevLED2IN = 0;
bool EtatClick1   = false;
bool EtatClick2   = false;

bool SimpleLed = false;   // la LED 3.3V de l'extrudeur

/* ---------------------------------------------------------- */
/* --------------  FONCTIONS BOUTON  ------------------------ */
/* ---------------------------------------------------------- */
// btn.getClicks() pour avoir le nombre de click 
// isButtonPressed() - Test if the button was pressed the last time it was sampled.
// isLongPressed() - Test it the button is in long-pressed state.
// getStartTime() - Returns the time of the first button press.
// getPrevTransitionTime() - Time of a previous transition. Returns startTime for the first transition.


void transitionHandler(EButton &btn) {      //triggered each time the button state changes from pressed to released, or back
}
void eachClickHandler(EButton &btn) {       //triggered each time the key  is released, unless it was in LONG_PRESSED state;
}
void singleClickHandler(EButton &btn) {     //triggered when there was exactly one click;
}
void doubleClickHandler(EButton &btn) {     //triggered when there were exactly two clicks;
}

void doneClickingHandler(EButton &btn) {    //triggered after all the clicks have been counted (use getClicks() to get the clicks count);
/* difference between EACH_CLICK and DONE_CLICKING: 
 * the first one is called each time the key is released (unless it was a long-press), 
 * while DONE_CLICKING is called once, at the end of clicks counting.
 */ 
	// on devrait tester ici si le bouton est un appui long ou pas. car à l'extinction immédiate du PI, cala risque d'être vu comme un seul click et générer un petit flash sur LED1 
  
  switch (btn.getClicks()) {
/* pour les 2 lumières commandées par le PI, on va considérer qu'il n'y a pas de PWM
 * Si le pi a sa commande à 0, le bouton peut allumer et éteindre la lumière
 * si le pi a sa commande à 1, le bouton complémente la sortie tant que la pin ne redescend pas à 0*/    
    case 1:
      if (EtatClick1) {
        EtatClick1 = false;
        digitalWrite(LedPowerOut1,0);
      } else {
        EtatClick1 = true;
        analogWrite(LedPowerOut1,LED1PWM);
      }
      break;

    case 2:
      if (EtatClick2) {
        EtatClick2 = false;
        digitalWrite(LedPowerOut2,0);
      } else {
        EtatClick2 = true;
        analogWrite(LedPowerOut2,LED2PWM);
      }
      break;

    case 3:
      if(SimpleLed) {
        SimpleLed = false;
        digitalWrite(LedOut,0);
      } else {
        SimpleLed = true;
        analogWrite(LedOut,LED4PWM);
      }
      break;

    case 4:
      if(ExtinctionPiParPi) {
        ExtinctionPiParPi = false;
        ExtinctionPiRefroidissement = false;
        ExtinctionPiImmediat = false;
        TimeExtinctionPiParPi = 0;
      } else {
        ExtinctionPiParPi = true;
        ExtinctionPiRefroidissement = false;
        ExtinctionPiImmediat = false;
        TimeExtinctionPiParPi = millis();
      }
      ValidationExtrudeur(4);
      break;

    case 5:
      if(ExtinctionPiRefroidissement) {
        ExtinctionPiParPi = false;
        ExtinctionPiRefroidissement = false;
        ExtinctionPiImmediat = false;
        TimeExtinctionPiRefroidissement = 0;
      } else {
        ExtinctionPiParPi = false;
        ExtinctionPiRefroidissement = true;

        ExtinctionPiImmediat = false;
        TimeExtinctionPiRefroidissement = millis();
      }
      ValidationExtrudeur(5);
      break;

    default:
      break;
  }
}

void pressStartHandler(EButton &btn) {      // triggered once, at the beginning of a long press (after a TRANSITION to pressed);
}

unsigned long t;
void duringPressHandler(EButton &btn) {     //triggered on each next tick() while in LONG_PRESSED state (not in the same cycle with LONG_PRESS_START);
	if ((unsigned long) (millis() - t) > 1000) {
		// Print once a second
	}
}
void pressEndHandler(EButton &btn) {        //triggered once, at the end of a long press (after a TRANSITION to released).
  ExtinctionPiParPi = false;      
  ExtinctionPiRefroidissement = false;
  ExtinctionPiImmediat = true;
  TimeExtinctionPiImmediat = millis();
}

/* ---------------------------------------------------------- */
/* ---------------  DEBUG PAR LED3 (non implémenté) --------- */
/* ---------------------------------------------------------- */
void LED3Debug(uint8_t NbLedBlink) {
  //if 
}

/* ---------------------------------------------------------- */
/* ----------  VALIDATION VISUEL PAR LED EXTRUDEUR ---------- */
/* ---------------------------------------------------------- */
void ValidationExtrudeur(uint8_t NbLedValidation) {
  digitalWrite(LedOut,0);
  delay(500);
  for (int i=1; i <= NbLedValidation; i++){
    digitalWrite(LedOut,1);
    delay(300);
    digitalWrite(LedOut,0);
    delay(300);
  }
  delay(200);
  if(SimpleLed)
    analogWrite(LedOut,LED4PWM);  
  else 
    digitalWrite(LedOut,0);
}


/* ---------------------------------------------------------- */
/* ----------------  SEQUENCE EXTINCTION PI ----------------- */
/* ---------------------------------------------------------- */
void SequenceResetPi() {
    digitalWrite(RPIReset,PIRESETE);
    delay(500);
    digitalWrite(RPIReset,PIFONCTIONNE);
    PiEteint = true;
    EtatClick1 = false;
    digitalWrite(LedPowerOut1,0);
    EtatClick2 = false;
    digitalWrite(LedPowerOut2,0);
    digitalWrite(LedPowerOut3,0);
    SimpleLed = false;
    digitalWrite(LedOut,0);
}


/* ---------------------------------------------------------- */
/* -------------------  SETUP  ------------------------------ */
/* ---------------------------------------------------------- */
void setup() {

  pinMode(LedPowerOut1, OUTPUT);
  digitalWrite(LedPowerOut1, 0);
  pinMode(LedPowerOut2, OUTPUT);
  digitalWrite(LedPowerOut2, 0);
  pinMode(LedPowerOut3, OUTPUT);
  digitalWrite(LedPowerOut3, 0);
  pinMode(LedOut, OUTPUT);
  digitalWrite(LedOut, 0);
  pinMode(RelayOut, OUTPUT);
  digitalWrite(RelayOut, 0);
  pinMode(RPIReset, OUTPUT);
  digitalWrite(RPIReset, 1);        // par défaut à 1, extinction en mettant la pin à la masse, comme un bouton poussoir

  pinMode(PshBut, INPUT_PULLUP);
  pinMode(LED1IN, INPUT_PULLUP);
  pinMode(LED2IN, INPUT_PULLUP);
  pinMode(RelayIN, INPUT_PULLUP);   // si le Attiny n'est pas connecté au RPI, il faut allumer le relais par défaut, mais il faut une pull-down. il faut en ajouter une en externe ou coller la pin à 0 par un jumper/fil

  pinMode(GPIOSHUTDOWNIN, INPUT);

 	// button.setDebounceTime(EBUTTON_DEFAULT_DEBOUNCE);		// not required if using default=50ms
	button.setClickTime(DureePression);				  // not required if using default=150ms = temps max entre un click releas et le click suivant pressed
	button.setLongPressTime(DureePressionLongue);	          // not required if using default=1sec

  // test du bouton poussoir pour savoir s'il est actif à la mise sous tension (bouton collant à 0 si actif car pull-up interne). Explication : 
  //  1° si l'utilisateur veut allumer l'imprimante au démarrage, sans passer par le PI, il appui sur le bouton.
  //  2° si l'utilisateur ne veut pas utiliser le PI, il peut coller le bouton à 0 indéfiniement

  ValidationExtrudeur(2); 
 #ifdef DEBUGMODE
  EtatBoutonBoot = false;           // pas d'activation du bouton poissoir au démarrage
  BoutonNotActivated = false;       // le bouton poissoir est actif
  GPIOSHUTDOWNSequenceVerif = WAITTOBOOT;    // indique que le PI a bien booté 
  BootTimerEnded = false;           // indique que le PI a booté et qu'il est actif et prêt à fonctionner
  button.attachDoneClicking(doneClickingHandler);
  button.attachLongPressEnd(pressEndHandler);
 #else
  delay(1500);                      // attente de 1 seconde pour que l'utilisateur puisse allumer PUIS appyer sur le bouton avec une seule main
  if (digitalRead(PshBut) == 1) 
    EtatBoutonBoot = false;
  else  {
    EtatBoutonBoot = true;
    digitalWrite(RelayOut,1);       // si le bouton a été activé, on allume l'imprimante au début du boot
    EtatRelais = true;
    delay(1500);                  //petit délais pour laisser le temps au relais de s'activer
  }

  BoutonNotActivated = true;
  while(millis() <= 5000) {        //délais de 5secondes pendant que le pi boot. 
                                  // Pendant ce temps on regarde si le bouton est relaché. S'il ne l'est pas au bout de cette tempo, on désactive le bouton

    if(digitalRead(PshBut) == 1) {    // si le bouton est relaché 
      if (BoutonNotActivated) {
        //	button.attachTransition(transitionHandler);
        //	button.attachEachClick(eachClickHandler);
          button.attachDoneClicking(doneClickingHandler);
        //	button.attachSingleClick(singleClickHandler);
        //	button.attachDoubleClick(doubleClickHandler);
        //	button.attachLongPressStart(pressStartHandler);
        //	button.attachDuringLongPress(duringPressHandler);
        button.attachLongPressEnd(pressEndHandler);
        BoutonNotActivated = false;         // on active le bouton une seule fois
        break;                              // on arrête la boucle ca plus de raison de vérifier l'état de la pin
                                            // si le bouton n'est pas désactivé dans ces 10 secondes, alors le bouton est inutilisable
      }
    }
  }

  while((millis() <= DelaisStartup) || (digitalRead(GPIOSHUTDOWNIN) == 0)) {   // pour que cela marche, il faut que le PI passe à 1 sa sortie relais pendant le boot. normalement 10s suffit
                                                              // si le PI met trop longtemps pour booter ou dès que le relais passe à 0 (le pi semble fonctionnel), on passe à la boucle principale
    if(BoutonNotActivated == false)
      button.tick();    // fonction qui vérifie le bouton
  }
  if (digitalRead(GPIOSHUTDOWNIN) == 1) {
    GPIOSHUTDOWNSequenceVerif = 1;        // si 1 alors cela veut dire que le PI a quand même booté
    BootTimerEnded = false;
  } else {
    BootTimerEnded = true;
    GPIOSHUTDOWNSequenceVerif = 0;        // etat par défaut de la séquence
  }
#endif
 
}

void loop() {
  if(BoutonNotActivated == false)
    button.tick();    // fonction qui vérifie le bouton

/* gestion des LEDs
 * si le pi demande à allumer alors que la sortie est déja allumée, alors ne fait rien, ou sinon l'allume
 * Si le pi demande à éteindre alors que la sortie est allumée, alors éteint */
  if(PiEteint == false) {               // sinon, si le PI est éteint alors on peut toujours allumer les LEDs par le bouton
    if(digitalRead(LED1IN) == 1) {
      if(!PrevLED1IN) {
          // changement d'état
          EtatClick1 = true;
          analogWrite(LedPowerOut1,LED1PWM);
        }
        PrevLED1IN = true;
    } else {
        if(PrevLED1IN) {
          // changement d'état
          EtatClick1 = false;
          digitalWrite(LedPowerOut1,0);
        }
        PrevLED1IN = false;
    }
    if(digitalRead(LED2IN) == 1) {
        if(!PrevLED2IN) {
          // changement d'état
          EtatClick2 = true;
          analogWrite(LedPowerOut2,LED2PWM);
        }
        PrevLED2IN = true;
    } else {
        if(PrevLED2IN) {
          // changement d'état
          EtatClick2 = false;
          digitalWrite(LedPowerOut2,0);
        }
        PrevLED2IN = false;
    }
  }

//gestion de l'extinction
/* si le bouton est validé par 4 clic, le pi sera éteint quand le relais sera éteint + DelaisSuicide. il faut attendre que le relais s'éteigne
 * si le bouton est validé par 5 clic, extinction immédiate après DelaisImprimanteSuiteSuicide 
 * si l'utilisateur veut éteindre tout de suite, un appui long */

  if(ExtinctionPiImmediat && (millis() - TimeExtinctionPiImmediat > DelaisExtinctionPiImmediat)) {      // extinction immédiate par appuis long
    SequenceResetPi();
    digitalWrite(RelayOut,0);
    EtatRelais = false;
    ExtinctionPiImmediat = false;
  } else {
    if(ExtinctionPiRefroidissement) {   // on éteint le pi d'abors et on attend un délais avant d'éteindre l'imprimante
      if(PiEteint == false) {
        SequenceResetPi();              // extinction du PI et des LEDs
      }
      if(millis() - TimeExtinctionPiRefroidissement > DelaisExtinctionPiRefroidissement) {
        digitalWrite(RelayOut,0);
        EtatRelais = false;
        ExtinctionPiRefroidissement = false;
      }
    } else {            // 
      if(PiEteint) {    // par défaut à false
        digitalWrite(RelayOut,0);
        EtatRelais = false;
      } else {          // si le PI est allumé, on va tester le mode suicide

        // prise en compte de l'entrée GPIO Shutdown > si le PI est éteint par le réseau, alors la pin tombe et l'Arduino peut éteindre l'imprimante (pas les lumières). On simule le bouto appuyé 4fois
        // on doit attendre que le PI ait booté car sinon, l'arduino va l'éteindre de suite. On fait donc uin pooling de la pin pour changer l'état de GPIOSHUTDOWNSequenceVerif 
        switch (GPIOSHUTDOWNSequenceVerif) {
          case STARTUP:     
            GPIOSHUTDOWNSequenceVerif = WAITTOBOOT;
            break;

          case WAITTOBOOT:
            if (digitalRead(GPIOSHUTDOWNIN) == 1) {
              delay(500);
              if (digitalRead(GPIOSHUTDOWNIN) == 1) {      // délais de 500m pour deglich, on reteste la pin pour savoir si elle est bien à 1, si le pi est bien démarré
                GPIOSHUTDOWNSequenceVerif = BOOTED;
              }
            }
            break;

          case BOOTED:
            if (digitalRead(GPIOSHUTDOWNIN) == 0) {
              delay(500);
              if (digitalRead(GPIOSHUTDOWNIN) == 0) {      // délais de 500m pour deglich, on reteste la pin pour savoir si elle est bien à 0, si le pi est bien off
                GPIOSHUTDOWNSequenceVerif = POTENTIALREBOOTED;
                TimePotentialReboot = millis();
              }
            }
            break;

          case POTENTIALREBOOTED:       // on attend une tempo avant de considérer octoprint réellement éteint
            if (millis() - TimePotentialReboot > DelaisReboot) {
              GPIOSHUTDOWNSequenceVerif = TURNEDOFF;
            } else {
              if (digitalRead(GPIOSHUTDOWNIN) == 1) {
                delay(500);
                if (digitalRead(GPIOSHUTDOWNIN) == 1) {      // délais de 500m pour deglich, on reteste la pin pour savoir si elle est bien à 1, si le pi est bien démarré
                  GPIOSHUTDOWNSequenceVerif = BOOTED;
                }
              }
            } 
          break;

          case TURNEDOFF:            //si octoprint est arrêté, on peut éteindre le pi
            ExtinctionPiParPi = true;                 // pour éteindre plus bas
            ExtinctionPiRefroidissement = false;
            ExtinctionPiImmediat = false;
            TimeExtinctionPiParPi = millis();
            TimeRelaisOff = millis();
            digitalWrite(RelayOut,0);
            EtatRelais = false;
            break;

          default:
            break;

        }  

 #ifdef DEBUGMODE                                 // mode DEBUG : gestion simple du relais, on recopie la pin du RPI
        if (digitalRead(RelayIN) == 1) {          // si allumage (état haut), on le fait immédiatement
          digitalWrite(RelayOut,1); 
          EtatRelais = true;
          EntreePiRelais = true;
          EntreePiRelaisPrev = true;
          TimeDeglichRelais = 0;
        } else {                            // si extinction, deglich de "DeglitchRelaisIn" ms (si le pi n'est pas présent, la pull-up interne va mettre la pin à '1')
          EntreePiRelais = false;
          if(EntreePiRelaisPrev != EntreePiRelais) {    // on est à la détection du front
            TimeDeglichRelais = millis();
          } else {                                      // on est à la détection du front
            if(millis() - TimeDeglichRelais > DeglitchRelaisIn) {
              if(EtatRelais)
                TimeRelaisOff = millis();         // on sauvegarde le temps de mise à 0 une seule fois, avant que l'état du relais ne soit changé. comme ca à la prochaine boucle ce timing ne sera pas continuellement modifié à l'heure courante
              digitalWrite(RelayOut,0);
              EtatRelais = false;
            }
          }
          EntreePiRelaisPrev = EntreePiRelais;
        }
 #else                                            // autre que mode DEBUG > il faut tenir compte de l'allumage sans pi puis le boot de ce dernier qui pourra reprendre la main
        // prise en compte du flag "EtatBoutonBoot" 
        // si le bouton n'a pas été activé au boot, le PI commande le relais normalement
        if((!EtatBoutonBoot) && (GPIOSHUTDOWNSequenceVerif != 2)) {       // si GPIOSHUTDOWNSequenceVerif = 2, ca veut dire que le RPI doit s'éteindre ou est déjà éteint
          if (digitalRead(RelayIN) == 1) {          // si allumage (état haut), on le fait immédiatement
            digitalWrite(RelayOut,1); 
            EtatRelais = true;
            EntreePiRelais = true;
            EntreePiRelaisPrev = true;
            TimeDeglichRelais = 0;
          } else {                            // si extinction, deglich de "DeglitchRelaisIn" ms (si le pi n'est pas présent, la pull-up interne va mettre la pin à '1')
            EntreePiRelais = false;
            if(EntreePiRelaisPrev != EntreePiRelais) {    // on est à la détection du front
              TimeDeglichRelais = millis();
            } else {                                      // on est à la détection du front
              if(millis() - TimeDeglichRelais > DeglitchRelaisIn) {
                if(EtatRelais)
                  TimeRelaisOff = millis();         // on sauvegarde le temps de mise à 0 une seule fois, avant que l'état du relais ne soit changé. comme ca à la prochaine boucle ce timing ne sera pas continuellement modifié à l'heure courante
                digitalWrite(RelayOut,0);
                EtatRelais = false;
              }
            }
            EntreePiRelaisPrev = EntreePiRelais;
          }
        } else {      // si le bouton a été activé au démarrage, le relais est en marche mais le PI a sa sortie désactivé (à 1). Si le PI a fini de booter dans les temps, il suffit que le PI veuille allumer le relais pour reprendre la main
                    // il est important que cette séquence n'arrête pas une impression en court
          if((BootTimerEnded == false) && (digitalRead(RelayIN) == 1))    // si le pi a bien booté dans les temsp et qu'il active sa sortie relais
            EtatBoutonBoot = false;         // le pi reprend la main
        }
#endif
        // Si le pi s'éteint de lui même en même temps que le relais
        if( ExtinctionPiParPi && (EtatRelais == false) && (millis() - TimeRelaisOff > DelaisExtinctionPiParPi)) {
          SequenceResetPi();
          EtatRelais = false;
          digitalWrite(RelayOut,0);          
          ExtinctionPiParPi = false;
        }
      }
    }
  }
}

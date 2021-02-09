/* Clock a 1MHz et nécessite la fonction millis*/
/* A changer si jamais la clock PWM du PI est trop rapide car l'arduino va recopier la pin de commande LED et doit le faire rapidement*/
/* Fonctionnement LED : 
 * si le pi n'a pas activé les lumière LED1 et LED2 alors l'utilisateur peut allumer éteindre avec respectivement un appui simple et un double appui
 * si le pi n'a pas activé la lumière LED1, un appui long allume LED1 et le relachement l'éteint
 * si l'utilisateur appui 3 fois, éteint/allume la LED de l'extrudeur
 * si l'utilisateur appui 4 fois, éteint le PI ou l'annule

 * Fonctionnement Relais/extinction
 * si le PI éteint l'imprimante, désactive immédiatement le relais en recopiant la pin
 * si l'utilisateur a validé l'extinction du PI par le bouton, alors éteindra le PI en activant la pin "RPIReset" après un certain délais (idéalement passage de 0 à 1 pui de 1 à 0)
 * Mais l'utilisateur peu désactiver cette extinction avec la même séquence (4 appui)
 * pour valider ou non le mode, la led de l'extrudeur doit s'allumer ou s'éteinde doucement pour dire si ca a été pris en compte. dans ce cas le précédent état de la LED de l'extrudeur doit être sauvegardé pour se remeetre dans l'état 
 * autre option si cablé (pas pour le moment) en cas d'ajout de BP su la pin "RPIReset" l'utilisateur peut éteindre directement le RPI. Dans ce cas l'ATTINY doit aussi mesurer cette pin  et couper l'imprimante si la pin est mies à 0 par l'utilisateur. 
*/

/*        PINOUT
 *        ------
 *          VDD  |      |GND
 *RelayIN/0/PA4  |      |PA3/10 /LedPowerOut3
 *LED2IN /1/PA5  |      |PA2/9  /LedPowerOut2
 *LED1IN /2/PA6  |      |PA1/8  /LedPowerOut1
 *RPIRst /3/PA7  |      |PA0/11 /ISP
 *TestClk/4/PB3  |      |PB0/7  /LedOut
 *PushBut/5/PB2  |      |PB1/6  /RelayOut
 *        ------*/

/* SOIC  Pin   ADC0  ADC1  DAC0  USART0  SPI0  TWI0  TCA0  TCBn    TCD0  CCL   
 * 10    PA0   AIN0  -     -     -       -     -     -     -       -     LUT0-IN0  >
 * 11    PA1   AIN1  -     -     TXDbis  MOSI  SDA   -     -       -     LUT0-IN1  > LedPowerOut1
 * 12    PA2   AIN2  -     -     RXDbis  MISO  SCL   -     -       -     LUT0-IN2  > LedPowerOut2
 * 13    PA3   AIN3  -     -     XCKbis  SCK   -     WO3   TCB1 WO -     -         > LedPowerOut3
 * 14    GND   -     -     -     -       -     -     -     -       -     -         >
 * 1     VDD   -     -     -     -       -     -     -     -       -     -         >
 * 2     PA4   AIN4  AIN0  -     XDIRbis SS          WO4           WOA   LUT0-OUT  > RelayIN
 * 3     PA5   AIN5  AIN1  -     -       -     -     WO5   TCB0 WO WOB             > LED2IN
 * 4     PA6   AIN6  AIN2  OUT                                                     > LED1IN
 * 5     PA7   AIN7  AIN3  -     -       -     -     -     -       -     LUT1-OUT  > RPIReset
 * 6     PB3   -     -     -     RXD     -     -     WO0bis                        > CLOCKOUTPUT
 * 7     PB2   -     -     -     TXD     -     -     WO2                           > PshBut
 * 8     PB1   AIN10 -     -     XCK     -     SDA   WO1                           > RelayOut
 * 9     PB0   AIN11 -     -     XDIR    -     SCL   WO0                           > LedOut
 */

#include "EButton.h"
//#define CLOCKOUTPUT     //pour faire sortir sur une pin la fréquence d'horloge divisé par 4
/* ---------------------------------------------------------- */
/* ------------------  HARDWARE  ---------------------------- */
/* ---------------------------------------------------------- */
/* Ajouter ATTINY1614 dans Arduino : 
 *  https://create.arduino.cc/projecthub/john-bradnam/using-the-new-attiny-processors-with-arduino-ide-612185
 * Ardui9no : MegaTinyCore>>ATTINY1614....
 * Bootloader : jtag2updi (megaTinyCore) à base d'un Arduino Nano Old bootloader
 * Info > BOD Volt:2,6V // Clock 1MHz // Mili/micro disabled // UART close to 3V
 * Librairies utilisées :  
 * Liens
 * - mode low power :http://www.technoblogy.com/show?2RA3
 * >> COMMENTAIRE
 * la clock principale sera le 32K interne
*/
/* ---------------------------------------------------------- *
 * -------------------  PINOUT  ----------------------------- *
 * ---------------------------------------------------------- */
#define LedPowerOut1        8  //PA1 > SORTIE transistor LED
#define LedPowerOut2        9  //PA2 > SORTIE transistor LED
#define LedPowerOut3        10 //PA3 > SORTIE transistor LED
#define LedOut              7  //PB0 > SORTIE simple LED pour extrudeur
#define RelayOut            6  //PB1 > SORTIE relais 24V/16A
#define RPIReset            3  //PA7 > SORTIE : sortie à la 0 pour éteinde le pi
//#define RPIReset            0b10000000
#define PshBut              5  //PB2 > ENTREE SEULE bonton poussoir en pull-up de commande pour interface homme Arduino
#define LED1IN              2  //PA6 > ENTREE DU PI
#define LED2IN              1  //PA5 > ENTREE DU PI
#define RelayIN             0  //PA4 > ENTREE DU PI
#define TestClk             4  //PB3 > SORTIE SI CONFIGUREE pour tester la fréquence d'horloge

EButton button(PshBut);     //bouton pressé quand à la masse

/* ---------------------------------------------------------- */
/* -----------------  CONSTANTES  --------------------------- */
/* ---------------------------------------------------------- */
#define DureePression                         984    // = ~300 * 32,768, choisi un nombre pair
#define DureePressionLongue                   3000   
#define DelaisExtinctionPiParPi               1000    // délais entre le moment ou l'imprimante est éteinte par le PI et le moment ou le PI est éteint par l'Arduino
#define DelaisExtinctionPiRefroidissement    120000  // délais entre le moment ou l'Arduino détecte une extinction et le moment ou il éteint l'imprimante (délais nécessaire pour refroidir le bed et l'extrudeur)
#define DelaisExtinctionPiImmediat            1000    // délais entre le moment ou l'imprimante est éteinte et le moment ou le PI est éteint (extinction immédiate)
#define DeglitchRelaisIn                      400     // délais pendant lequel l'entrée venant du pi doit rester à 0 avant l'extinction réelle du relais de l'imprimante
#define MCLK_PRESC_2    0x00
#define MCLK_PRESC_4    0x01
#define MCLK_PRESC_8    0x02
#define MCLK_PRESC_16   0x03
#define MCLK_PRESC_32   0x04
#define MCLK_PRESC_64   0x05
#define MCLK_PRESC_6    0x08
#define MCLK_PRESC_10   0x09
#define MCLK_PRESC_12   0x0A
#define MCLK_PRESC_24   0x0B
#define MCLK_PRESC_48   0x0C

/* ---------------------------------------------------------- */
/* -----------------  VARIABLES  ---------------------------- */
/* ---------------------------------------------------------- */
bool BoutonInactif = true;      // si le bouton n'est pas actif, true, pour que le PI change l'éclairage
bool ExtinctionPiParPi = false;      // TRUE, si l'utilisateur a demandé à éteindre le PI en même temps que l'imprimante, 
bool ExtinctionPiRefroidissement = false;      // TRUE, si l'utilisateur a demandé à éteindre le PI après un délais de refroidissement, 
bool ExtinctionPiImmediat = false;      // TRUE, si l'utilisateur a demandé à éteindre le PI immédiatement 
bool EntreePiRelais = false;            //variables pour le deglich de l'entrée relais venant du PI
bool EntreePiRelaisPrev = false;
bool EtatRelais = false;                // donne l'état du relais

bool EntreePiLED1 = false;            //variables pour le deglich de l'entrée LED1 venant du PI
bool EntreePiLED1Prev = false;
bool EntreePiLED2 = false;            //variables pour le deglich de l'entrée LED2 venant du PI
bool EntreePiLED2Prev = false;


bool PiEteint = false;          // TRUE, si le pi a été éteint par l'Arduino 
unsigned long TimeExtinctionPiParPi=0;
unsigned long TimeExtinctionPiRefroidissement=0;
unsigned long TimeExtinctionPiImmediat=0;

unsigned long TimeDeglichRelais=0;
unsigned long TimeRelaisOff=0;

uint8_t PrevLED1IN = 0;
uint8_t PrevLED2IN = 0;
bool EtatClick1   = false;
bool EtatClick2   = false;

bool SimpleLed = false;   // la LED 3.3V de l'extrudeur
bool Led24V1 = false;     // Sortie LED 24V N°1
bool Led24V2 = false;     // Sortie LED 24V N°2
bool Led24V3 = false;     // Sortie LED 24V N°3

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
        digitalWrite(LedPowerOut1,1);
      }
      break;
    case 2:
      if (EtatClick2) {
        EtatClick2 = false;
        digitalWrite(LedPowerOut2,0);
      } else {
        EtatClick2 = true;
        digitalWrite(LedPowerOut2,0);
      }
      break;
    case 3:
      if(SimpleLed) {
        SimpleLed = false;
        digitalWrite(LedOut,0);
      } else {
        SimpleLed = true;
        digitalWrite(LedOut,1);
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
/* -------------------  SETUP  ------------------------------ */
/* ---------------------------------------------------------- */
void setup() {
  // Enable writing to protected register
  //no need for low power clock
  /*_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCULP32K_gc);
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (MCLK_PRESC_64 << 1) | 1);
  _PROTECTED_WRITE(CLKCTRL.OSC20MCTRLA, 0x00);*/

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
  digitalWrite(RPIReset, 0);
#ifdef CLOCKOUTPUT
  pinMode(TestClk, OUTPUT);
  digitalWrite(TestClk, 0);
#else
  pinMode(TestClk, INPUT_PULLUP);
#endif
  pinMode(PshBut, INPUT_PULLUP);
  pinMode(LED1IN, INPUT_PULLUP);
  pinMode(LED2IN, INPUT_PULLUP);
  pinMode(RelayIN, INPUT_PULLUP);

  //Création boucle Timer RTC
  /*while (RTC.STATUS > 0)    //nécessaire ?
    ;
  RTC.CTRLA = 0;
  RTC.CMP = DureePression >> 1; //comparaison /2 mais ne servira pas à grand chose
  RTC.PER = DureePression; //top avant remise à 0
  RTC.INTCTRL = 0 ; // pas d'inter 
  RTC.CLKSEL = 0; // clk = 32k interne
  RTC.CNTL = 0;   //reset du compteur
  RTC.CNTH = 0;   //reset du compteur*/

#ifdef CLOCKOUTPUT
  BlinkClock();
#endif

/* //initialise les paramètres pour ne rien faire au démarrage de la boite/carte
  RTC.CNTL = 0;   //reset du compteur
  RTC.CNTH = 0;   //reset du compteur
  RTC_ON(); */

 	// button.setDebounceTime(EBUTTON_DEFAULT_DEBOUNCE);		// not required if using default=50ms
	// button.setClickTime(EBUTTON_DEFAULT_CLICK);				  // not required if using default=150ms = temps max entre un click releas et le click suivant pressed
	button.setLongPressTime(DureePressionLongue);	          // not required if using default=1sec
  
//	buttogn.attachTransition(transitionHandler);
//	button.attachEachClick(eachClickHandler);
	button.attachDoneClicking(doneClickingHandler);
//	button.attachSingleClick(singleClickHandler);
//	button.attachDoubleClick(doubleClickHandler);
//	button.attachLongPressStart(pressStartHandler);
//	button.attachDuringLongPress(duringPressHandler);
	button.attachLongPressEnd(pressEndHandler);
}

void loop() {
  // la boucle RTC tourne toute les ~300 msec
  //RTC_WAIT();
#ifdef CLOCKOUTPUT
  BlinkLEDtest();   // test la fréquence de l'horloge en faisant bagoter cette pin inutilisée
#endif
  button.tick();    // fonction qui vérifie le bouton

/* gestion des LEDs
 * si le pi demande à allumer alors que la sortie est déja allumée, alors ne fait rien, ou sinon l'allume
 * Si le pi demande à éteindre alors que la sortie est allumée, alors éteint */
  if(digitalRead(LED1IN) == 1) {
      if(!PrevLED1IN) {
        // changement d'état
        EtatClick1 = true;
        digitalWrite(LedPowerOut1,1);
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
        digitalWrite(LedPowerOut2,1);
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

//gestion de l'extinction
/* si le bouton est validé par 4 clic, le pi sera éteint quand le relais serq éteint + DelaisSuicide. il faut attendre que le relais s'éteigne
 * si le bouton est validé par 5 clic, extinction immédiate après DelaisImprimanteSuiteSuicide 
 * si l'utilisateur veut éteindre tout de suite, un appui long */
  if(ExtinctionPiImmediat && (millis() - TimeExtinctionPiImmediat > DelaisExtinctionPiImmediat)) {
    digitalWrite(RelayOut,0);
    EtatRelais = false;
    digitalWrite(RPIReset,1);
    delay(500);
    digitalWrite(RPIReset,0);
    ExtinctionPiImmediat = false;
    PiEteint = true;
  }
  if(ExtinctionPiRefroidissement) {   // on éteint le pi d'abors et on attend un délais avant d'éteindre l'imprimante
    if(PiEteint == false) {
      digitalWrite(RPIReset,1);
      delay(500);
      digitalWrite(RPIReset,0);
      PiEteint = true;
    }
    if(millis() - TimeExtinctionPiRefroidissement > DelaisExtinctionPiRefroidissement) {
      digitalWrite(RelayOut,0);
      EtatRelais = false;
      ExtinctionPiRefroidissement = false;
    }
  }

  if (digitalRead(RelayIN)) {          // si allumage, on le fai immédiatement
    digitalWrite(RelayOut,1); 
    EtatRelais = true;
    EntreePiRelais = true;
    EntreePiRelaisPrev = true;
    TimeDeglichRelais = 0;
  } else {                            // si extinction, deglich de "DeglitchRelaisIn" ms
    EntreePiRelais = false;
    if(EntreePiRelaisPrev == EntreePiRelais) {    // on est à la détection du front
      TimeDeglichRelais = millis();
    } else {                                      // on est à la détection du front
      if(millis() - TimeDeglichRelais > DeglitchRelaisIn) {
        digitalWrite(RelayOut,0);
        EtatRelais = false;
        TimeRelaisOff = millis();
      }
    }
    EntreePiRelaisPrev = EntreePiRelais;
  }
 
  if(ExtinctionPiParPi && (EtatRelais == false) && (millis() - TimeRelaisOff > DelaisExtinctionPiParPi)) {
    digitalWrite(RPIReset,1);
    delay(500);
    digitalWrite(RPIReset,0);
    ExtinctionPiParPi = false;
    PiEteint = true;
  }
}

static inline void BlinkClock() {     // bagotte la pin 4 fois pour mesurer la fréquence d'horloge
                                      //pin4/PB3 > pour tester la fréquence d'horloge
  PORTB.OUTTGL = 0b00001000;
  PORTB.OUTTGL = 0b00001000;
  PORTB.OUTTGL = 0b00001000;
  PORTB.OUTTGL = 0b00001000;
  PORTB.OUTTGL = 0b00001000;
  PORTB.OUTTGL = 0b00001000;
  PORTB.OUTTGL = 0b00001000;
  PORTB.OUTTGL = 0b00001000;
}
static inline void RTC_ON() {
  RTC.CTRLA = B00000001 ;
}
static inline void RTC_OFF() {
  RTC.CTRLA = B00000000 ;
}
static inline void RTC_RESET() {
  RTC.INTFLAGS = B00000011 ;
}
static inline void RTC_WAIT() {
  while(RTC.INTFLAGS == 3)    //attente de la fin de la boucle
    ;
  RTC.INTFLAGS = B00000011 ;  // RTC_RESET
}
static inline void RTC_DEMI_WAIT() {
  while(RTC.INTFLAGS == 2)    //attente de la moitié de la boucle
    ;
  RTC.INTFLAGS = B00000010 ;      // reset juste le flag de COMP, pas le PER (Overflow)
}

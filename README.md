# CrealityLight 
## or how to connect a RPI/Ocroprint/Octoscreen and add some light 

This project does help how to connect a RPI to the octoscreen with a LED powerstage and a power relay.
This board does make the interface between the RPI and some MOS. 
But I have some doubt : 
- does the output of the relay goes to 0 when it is OFF ?
- how to turn on/off some LEDs manually without going to the octorpint interface, for a quick control ?
- how to turn off the RPI when the print is ended (the RPI can turn off the printer but may not turn it off) ?

I could have directly connected the RPI to the MOS, but I have decided to add an ATTINY1614 to add some flexibility.
I have added a puchbutton for some local interraction :
1. the pushbutton can turn on/off 3 different LED sting
1. it can also turn on/off a local LED (3V3) close to the extruder, to see the filament hole when we change it
1. it can force the RPI to be turned OFF locally :
   1. immediately (turn off the printer also)
   1. after a cooldown delay : the RPI is turned off immedately and the printer after some time (delay fixed in the ATTINY firmware)
   1. after a print, when the RPI does turn off the printer because of the idle time.

## ATTINY connection

The Attiny does have to communicate with the PI. it has then to work under a 3.3V. The PRI does then provide a 3.3V to the ATTINY
The RPI is powered from the main printer AC/DC through a simple USB DC/DC. I have used a mEZS91202A from MPS (MonolithicPowerSystem) becaue of its simple connection. It can also deliver 2.5A : 
- https://www.monolithicpower.com/en/mezs91202a.html
- you can also consider a mEZD71202A-G for a 2A rail from 24V input (maybe it can also handle 3A)

The scheamtic and layout is in this project


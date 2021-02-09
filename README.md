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
1. it can also turn on/off a local LED (3V3) close to the extruder, to see the filament hole when we change it (this LED is not driven by the pi, only by the user because it is used to bring some light when I am changing the filament (I am not ont the onctoprint web interface)) 
1. it can force the RPI to be turned OFF locally :
   1. immediately (turn off the printer also)
   1. after a cooldown delay : the RPI is turned off immedately and the printer after some time (delay fixed in the ATTINY firmware)
   1. after a print, when the RPI does turn off the printer because of the idle time.
I also connect an emergency push button (direct connection) and plan to use a SmartFilamentSensor

## ATTINY connection

The Attiny does have to communicate with the PI. it has then to work under a 3.3V. The PRI does then provide a 3.3V to the ATTINY
The RPI is powered from the main printer AC/DC through a simple USB DC/DC. I have used a mEZS91202A from MPS (MonolithicPowerSystem) becaue of its simple connection. It can also deliver 2.5A : 
- https://www.monolithicpower.com/en/mezs91202a.html
- you can also consider a mEZD71202A-G for a 2A rail from 24V input (maybe it can also handle 3A)

The scheamtic and layout is in this project
The connection does use the end of the GPIO connector, to be able to use a touchscreen display for octodash/octoscreen : 
RPI GPIO | RPI BCM | ATTINY pin | ATTINY port | Signal description
-------- | ------- | ---------- | ----------- | ------------------ 
36 |  16|  3|  PA5| LED2IN
37 |  26|  2|  PA4| RelayIN : output of the PI to turn ON/OFF the relay
38 |  20|  4|  PA6| LED1IN
39 |  GND|  14|  GND| 
40 |  21|  5|  PA7| RPIReset : input for the PI, to be turned off

## Octoprint pluggins and config
### Autoscroll (0.0.3)
### Bed Visualizer (1.0.0)
### BetterHeaterTimeout (1.3.0)
### DisplayLayerProgress Plugin (1.25.3)
### Emergency Stop Simplified (0.1.1)
### Exclude Region (0.3.0)
### Floating Navbar (0.3.4)
### GPIO Shutdown (1.0.3)
- this pluggin would have worked perfectly but it seems to mix at start-up the BCM and GPIO pins. to get it funcional, I have to select in the setup the GPIO number that I want (and not the BCM one) and save. Then the lED is turned ON immediately. Consider to use instead a simple script/service that is available on the web and out of octoprint. I wil otherwise change the Jinja config file to link the BCM pin to the GPIO.
Let's try with this config (https://www.instructables.com/Simple-Raspberry-Pi-Shutdown-Button/) to get a shutdown on level 1 because the ATTINY when OFF (not powered-up) will pull the pin low :
gpio_pin_number=21
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_pin_number, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
try:
    GPIO.wait_for_edge(gpio_pin_number, GPIO.RISING)
    os.system("sudo shutdown -h now")
except:
    pass
GPIO.cleanup()

### LED Strip Control (0.3.7)
### LightsOut (0.1.2)
### Multi Colors (1.0.17) 
### OctoLight (0.1.1)
### PSU Control (0.1.11)
### Simple Emergency Stop (1.0.4) 
### Smart Filament Sensor (1.1.5.3) 
### Tab Order (0.5.12) 
### not acivated - Octoprint-Display-ETA (1.1.3) 

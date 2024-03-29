# CrealityLight V2 !!
## or how to connect a RPI/Ocroprint/Octoscreen and add some light 
### Read the wiki pages to get few more details.
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
31 |  6|  4|  PA6| LED1IN
32 |  12|  3|  PA5| LED2IN
33 |  13|  2|  PA4| RelayIN : output of the PI to turn ON/OFF the relay
34 |  GND|  14|  GND| 
36 |  16|  5|  PA7| RPIReset : input for the PI, to be turned off
37 |  26|  na|  na| Emergency stop : push button to turn off a print if something goes wrong
38 |  20|  7|  PB2| OctoprintReady: signal from octoprint saying that it is alive
40 |  21|  na|  na| SmartSensor input : logic signal from a filament sensor

On the ATTINY board connector this mean :
ATTINY pin | ATTINY port | Arduino port | Signal description
---------- | ----------- | ------------ | ------------------
1 | 3.3V |  | power from a LDO on the board
2 | PA4 | 0 | RelayIN : signal from the PI
3 | PA5 | 1 | LED2IN : signal from the PI
4 | PA6 | 2 | LED1IN : signal from the PI
5 | PA7 | 3 | PI Reset : signal from the arduino to the PI to turn it off
6 | PB3 | 4 | PushButton : loacl button to select the action : turn ON/OFF the led or select which PI turn-off senario
7 | PB2 | 5 | OctoPrint Ready : signal from the PI saying that octoprint is ready (set to 1 when active)
8 | PB1 | 6 | LEDOUT : simple 3.3V LED, used for the Extruder
9 | PB0 | 7 | LEDPOWEROUT1 : MOS to 24V to drive one LED string
10 | UPDI | na | 
11 | PA1 | 8 | LEDPOWEROUT2 : MOS to 24V to drive one LED string
12 | PA2 | 9 | LEDPOWEROUT3 : MOS to 24V to drive one LED string
13 | PA3 | 10 | RELAYOUT : MOS to 24V to drive the printer relay
14 | GND |  | 

## Octoprint pluggins and config (see also the wiki)
quick note : the pluggin that turn off the RPI on a GPIO state has to be very carefully installed. I would recommend to disable the pin each time you disable it. otherwise, when you enable the pluggin in the octoprint "Pluggin Manager" it will immediately set the pin and activate it. And then your PI may turn off immediately becasue the pin is not driven

quick note : the pluggins can use GPIO or BCM numbering. Some give the choice like PSU Control. I would think that there is a mix when each pluggin does try to name the pins with different method. For the moment, the BCM method seems to be more functional.  

### Autoscroll (0.0.3)
### Bed Visualizer (1.0.0)
### BetterHeaterTimeout (1.3.0)
### Creality-2x-temperature-reporting-fix (0.0.4)
### DisplayLayerProgress Plugin (1.25.3)
### Emergency Stop Simplified (0.1.1) 

By default the pluggin does apply a pull-up or pull-down and check if the switch is open. If the button is set to GND, the pluggin does set a pull-up and will stop ONLY if the voltage is pulled-up by the internal pull-up resistor. By checking with an multimeter, I was no able to see a 3.3V on the pin when the switch is activated (open). I had a look then on the log (cd .octoprint/logs and then go really down to nano octoprint.log) and see the message : 
```
File "/home/pi/oprint/lib/python3.7/site-packages/octoprint_emergencystopsimplified/__init__.py", line 59, in _setup_button
    GPIO.setmode(GPIO.BCM)
ValueError: A different mode has already been set!
```
I tried then to change the mode from BCM to BOARD. I change the pin number to 37 (value in GPIO mode). A complete restart and I have been able to see that my pull-up was then present (I can see the voltage going up to 3,3V when the button is activated). During the print the printer goes immediately offline.

Note : Alexiri pluggin has the choice to turn off the printer or make a pause : https://github.com/alexiri/Emergency_stop_simplified/blob/master/octoprint_emergencystopsimplified/

Note : crysxd pluggin has the choice to select if the action is performed when the switch is going to open or closed state.
> would need to add an option to select BCM OR GPIO mode. And add a test mode like for the SmartFilament Sensor that has a specific "motion" test

My configuration: pin = 32 (GPIO pinout because I change from BCm to GPIO in the __init__.py) and button connected to ground.

### Exclude Region (0.3.0)
### Floating Navbar (0.3.4)
### Fullscreen Plugin (0.0.6)
### GPIO Shutdown (1.0.3)
- this pluggin would have worked perfectly but it seems to mix at start-up the BCM and GPIO pins. to get it funcional, I have to select in the setup the GPIO number that I want (and not the BCM one) and save. Then the LED is turned ON immediately. Consider to use instead a simple script/service that is available on the web and out of octoprint. I wil otherwise change the Jinja config file to link the BCM pin to the GPIO.
- the pluggin location is in "~/oprint/lib/python2.7/site-packages/octoprint_GPIOShutdown"
- to get it functional, I have changed manually the JINJA2 file to allow the selection of pins up to 40. By doing this, I can select a high pin number.
- My setup : shutdown on pin GPIO36 / PIN LED : GPIO 38 with default debounce time.
- To get this, i have to change the jinja file the selection menu for both the led and the shutdown pin:
```
cd ~/oprint/lib/python2.7/site-packages/octoprint_GPIOShutdown/templates/
nano GPIOShutdown_settings.jinja2
                                <option value="-1">Disabled</option>
                                <option value="2">BCM 2</option>
                                <option value="3">BCM 3</option>
                                <option value="4">BCM 4</option>
                                <option value="5">BCM 5</option>
                                <option value="6">BCM 6</option>
                                <option value="7">BCM 7</option>
                                <option value="8">BCM 8</option>
                                <option value="9">BCM 9</option>
                                <option value="10">BCM 10</option>
                                <option value="11">BCM 11</option>
                                <option value="12">BCM 12</option>
                                <option value="13">BCM 13</option>
                                <option value="14">BCM 14</option>
                                <option value="15">BCM 15</option>
                                <option value="16">BCM 16</option>
                                <option value="17">BCM 17</option>
                                <option value="18">BCM 18</option>
                                <option value="19">BCM 19</option>
                                <option value="20">BCM 20</option>
                                <option value="21">BCM 21</option>
                                <option value="22">BCM 22</option>
                                <option value="23">BCM 23</option>
                                <option value="24">BCM 24</option>
                                <option value="25">BCM 25</option>
                                <option value="26">BCM 26</option>
                                <option value="27">BCM 27</option>
                                <option value="3">GPIO 3</option>
                                <option value="5">GPIO 5</option>
                                <option value="7">GPIO 7</option>
                                <option value="8">GPIO 8</option>
                                <option value="10">GPIO 10</option>
                                <option value="11">GPIO 11</option>
                                <option value="12">GPIO 12</option>
                                <option value="13">GPIO 13</option>
                                <option value="15">GPIO 15</option>
                                <option value="16">GPIO 16</option>
                                <option value="18">GPIO 18</option>
                                <option value="19">GPIO 19</option>
                                <option value="21">GPIO 21</option>
                                <option value="22">GPIO 22</option>
                                <option value="23">GPIO 23</option>
                                <option value="24">GPIO 24</option>
                                <option value="26">GPIO 26</option>
                                <option value="29">GPIO 29</option>
                                <option value="31">GPIO 31</option>
                                <option value="32">GPIO 32</option>
                                <option value="33">GPIO 33</option>
                                <option value="35">GPIO 35</option>
                                <option value="36">GPIO 36</option>
                                <option value="37">GPIO 37</option>
                                <option value="38">GPIO 38</option>
                                <option value="40">GPIO 40</option>
```
Backup : a local service routine:  
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

### LED Strip Control (0.3.7) (not installed)
### LightsOut (0.1.2)
### Multi Colors (1.0.17) 
### OctoLight (0.1.1)
> Configuration : Light pin = 38
### PSU Control (0.1.11)
to be changed, checked : For some reason, the pluggin does not work if the pin mode is GPIO. But it does work well when the BCM mode is selected. you need to change again th python to select the "BOARD" mode instead of "BCM"
> Configuration : GPIO mode : Board > GPIO pin > 37 // internal sensing (5s)
### Simple Emergency Stop (1.0.4) 
### Smart Filament Sensor (1.1.5.3)
Board mode / pin 40 with the button "enable sensor" checked > command M600 (to be tested), detection mode : distance set at 15mm (default value)
and then tested with the small test button
### Tab Order (0.5.12) 
### not acivated - Octoprint-Display-ETA (1.1.3) 
### WebcamTab (0.2.0)

# Mini rover mini swarm

Using the pololu romi platform to create a swarm of few rovers.
Documenting the assembly, setup and usage of some simple, basic and more or less cheap rover platform such as the pololu romi to be controlled by an autopilot (ex ardurover) and evolve in a swarm.

## Rover platform

For all options (pololu romi or alternatives):
- autopilot board compatible with the chosen autopilot (ardupilot or PX4), we chose the mateksys F405-WMO, the [h743-mini](https://www.mateksys.com/?portfolio=h743-mini) is fine too.

### Romi

BOM (can be adjusted)
- romi chassis
- [power distribution](https://www.pololu.com/product/3541) board or the [motor driver + power board](https://www.pololu.com/product/3543) or the [controller board](https://www.pololu.com/product/3544) 
- [encoder pair for romi motors ](https://www.pololu.com/product/3542)
- NiMH batteries
- motor driver (if not using the controller board)

[Assembly guide:](https://www.pololu.com/docs/0J68/all)

Other references:
- https://ardupilot.org/rover/docs/common-autopilots.html
- https://discuss.ardupilot.org/t/ardurover-with-the-pololu-romi/41991


### Cheap platform



## autopilot

### control modes

The [follow mode](https://ardupilot.org/rover/docs/follow-mode.html) can be used for a basic swarm like behaviour, though it doesn't include any form of self-decision: vehicles follow each other and in order, with one vehicle in front.

### Firmware


This page says to use this firmware:
https://www.mateksys.com/?portfolio=f405-wmo#tab-id-2

https://firmware.ardupilot.org/Rover/stable/MatekF405-TE/




## connections

Power distribution board
By dfault the power distribution board supplies voltage from all the 6 batteries of the ROMI, this can be changed but having enough voltage is best to power directly the autopilot.

![power_dist_pinout](img/romi_power_dist_pinout_pololu_0J7290.jpg)

**Wheel encoder.**

VCC can be 3.5 V to 18 V

*Important*
The Mateksys F405-WMO flight controller does not have physical, onboard pull-up resistors, neither does the wheel encoder for ROMI. They must be installed or the wheel encoder signal will be too weak for the FC to detect it.
Solder 4.7k resistor between the output A and B and VCC for each encoder.
That is, one 4.7k resistor from output A to VCC and another 4.7k from output B to VCC for left encoder, repeat for the right one.

https://ardupilot.org/rover/docs/wheel-encoder.html

Connect the encoder VCC to one of the Vx pin on the autopilot.

The AP we use has 3.3V logic levels but it is 5V tolerant.


Motor driver

It requires 2 separate supplies, one to the logic (1.8V ~ 7V) and one for the power to the motors (2~11V). The logic voltage can be obtained from the autopilot, or via a voltage converter.

Note that the motors are positioned in a mirror image of each other so if connecting everything exactly on the left and right, motors will spin in opposite directions when increasing the throttle. If this happens, simply inverse M1 with M2 on one of the encorder board (or on the driver output).
The same concept goes if motors are spinning in reverse (rover moving forward when it should move backward).

**RC receiver.**

A futaba receiver was used and pairing and connection varies depending on the RC brand and protocol.

This is an sbus based Rx, connect to the mateksys to the SBUS pin, and a 5V power pin such as Vx or the 4V5 pin which is powered when the FC is connected to USB to a computer.

## Autopilot setup

Calibrate the radio in QGC or MP.

Setup the various parameters

Everytime one or more parameters is set, the FC has to be rebooted in order for the parameter to be taken into account.
For example in a newly flashed FC, RELAYx_FUNCTION is set to 0 and no other RELAY related parameters will appear.
In order to access to other RELAY parameters such as RELAYx_PIN, etc, the RELAYx_FUNCTION has to be set and the FC rebooted.

We will use the motor driver in phase-enable mode (in motor driver language; ithis corresponds to "brushed with relay" in the ardupilot [wiki](https://ardupilot.org/rover/docs/common-brushed-motors.html#common-brushed-motors)), and the connections follows the schematic from pololu.
"Brushed With Relay” is for brushed motor drivers that use a relay pin to indicate whether it should rotate forward or backward. 

The type of control for the romi is called skid steering. The settings below are made for this.

https://ardupilot.org/rover/docs/rover-motor-and-servo-configuration.html#skid-steering



Summary of all settings


RELAY1_FUNCTION -> 5
RELAY1_PIN -> 50
RELAY2_FUNCTION -> 6
RELAY2_PIN -> 51

WENC_TYPE -> 1 (quadrature)
WENC2_TYPE -> 1 (quadrature)

# MOT_PWM_TYPE


MOT_PWM_TYPE -> 3 (BrushedWithRelay)

*SERVO*

Servo parameters dictates whether a given pin acts as PWM or GPIO.
They must be set properly.

See below for how to set them on a per-function basis.

*RELAY_FUNCTION*

Relay are used to select the rotation direction of the motors on the motor driver.
Search for relay in the search box of QGC or MP.
Set as follows:

RELAY1_FUNCTION -> 5
RELAY2_FUNCTION -> 6

For this to work we also have to set the corresponding servo function to GPIO:

SERVO7_FUNCTION -> -1 (GPIO)
SERVO8_FUNCTION -> -1 (GPIO)

Reboot

Then set relay pins:
RELAY1_PIN -> 56
RELAY2_PIN -> 57

[!] The value for these parameters is HW dependent, check the FC datasheet and double check with the hwdef.dat file corresponding to that particular FC.
Note also that these are connected to S1 and S2 pins on the FC. 
[!] it is adviced not to mix pins dedicated to GPIO (ex, relay function) and pins dedicated to PWM.


For the MatekSys F405-WMO, the corresponding hwdef.dat is [here](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/MatekF405-TE/hwdef.dat)


*WENC (Wheel encoder)*

Related [ardurover doc](https://ardupilot.org/rover/docs/wheel-encoder.html).
set WENC_TYPE  to 1 (quadrature) and reboot the FC to enable other parameters and the second wheel encoder.

WENC_TYPE -> 1 (quadrature)

reboot so that more parameters and the second wheel encoder appears.

WENC2_TYPE -> 1 (quadrature)

Set the corresponding servo function to GPIO:

SERVO3_FUNCTION -> -1 (GPIO)
SERVO4_FUNCTION -> -1 (GPIO)
SERVO5_FUNCTION -> -1 (GPIO)
SERVO6_FUNCTION -> -1 (GPIO)

set EK3_SRC1_VELXY to 7 (WheelEncoder)

## command the rover with mavlink messages

### using mavproxy

Install mavproxy

Turn on the main power , plug the FC usb to the computer.

    mavproxy.py --master=/dev/ttyACM0 --baudrate=115200

To connect via an AP, see picoW AP code, use:

    mavproxy.py --master=udpout:192.168.4.1:14550

`192.168.4.1` is the picoW's IP address and `14550` the port for udp


Lots of messages will be shown in the console, which can be inconvienient when typing malink commands, so send these message to the gui console by typing:

    module load console

Make sure EK3_SRC1_VELXY is set to 7 (to make use of wheel odometry)

Either in QGC or using a mavlink message:

params set ek3_src1_velxy 7 

(tip: tab completion works, and if the parameter's name is unsure, type:

`param set ek3_src1*`, which will show:

    EK3_SRC1_POSXY   0           # None
    EK3_SRC1_POSZ    0           # None
    EK3_SRC1_VELXY   7           # WheelEncoder
    EK3_SRC1_VELZ    0           # None
    EK3_SRC1_YAW     0           # None

We need to set the GPS origin:

    module load message
    message SET_GPS_GLOBAL_ORIGIN 0 -353621474 1491651746 600000 0

Change to guided mode and arm:

    mode guided
    arm throttle

Then send a move forward message:

    message SET_POSITION_TARGET_LOCAL_NED 0 0 0 7 3580 0.6 0 0 0 0 0 0 0 0 0 0


Simple test: spin the wheels for a given distance or velocity:





Ensure 

### using program (C or python)

A program is used to send specific commands.

Python:

1) connect to the pico W AP
2) connect to rover with :
mavproxy.py --master=udpout:192.168.4.1:14550

3) in mavproxy, run:

        module load console
        module load message
        message SET_GPS_GLOBAL_ORIGIN 0 -353621474 1491651746 600000 0
        mode guided
        arm throttle

4) run the python script to move forward:
    
    py move_fwd.py


## send telemetry via a companion computer.

We chose the raspberry pi picoW for this.

We'll set up a full MAVLink communication between the rover and QGroundControl (QGC) using the Raspberry Pi Pico W as a UDP bridge.

- Integrate MAVLink into the Pico W code (to encode/decode MAVLink messages).
- Forward MAVLink messages between the flight controller (FC) and QGC via UDP.
- Ensure QGC is configured to connect to the Pico W's UDP endpoint.


## Useful references:

- [list of all rover parameters](https://ardupilot.org/rover/docs/parameters.html)
- 

## Abreviations

AP: AutoPilot (=FC)
FC: Flight controller


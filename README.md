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

Wheel encoder.

VCC can be 3.5 V to 18 V

https://ardupilot.org/rover/docs/wheel-encoder.html

Motor driver

It requires 2 separate supplies, one to the logic (1.8V ~ 7V) and one for the power to the motors (2~11V). The logic voltage can be obtained from the autopilot, or via a voltage converter.

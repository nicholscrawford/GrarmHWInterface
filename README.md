# GrarmHWInterface
Ros Package for Grarm Hardware Interface

A simple implementation of a hardware interface for "Green Arm". It definately has it's flaws, as I wrote it really before knowing c++. It works though, but relies on a high refresh rate rather than actual pos/vel. In my next robotic arm project, I'd structure it differently, as a node that can interface with the prebuilt moteus python interface. 

The implementation is all in hwinterface.h, in https://github.com/nicholscrawford/GrarmHWInterface/blob/main/src/grarm/src/hwinterface.h. This was just keep everything as simple as possible, but is definately not ideal. As a starting place for how a robot arm can be built with the Moteus brushless motor controller, it can bea useful reference.

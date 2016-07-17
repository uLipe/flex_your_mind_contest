# flex_your_mind_contest
This is the firmware for freedom board K82 which implements a motion estimator targeted to fertilizer sensoring for precision agriculture application.

#Build instructions
you must have the make build utility, navigate to application folder, here you wil find the makefile with a custom recipe to build the binary.
Once the file found in this folder open a command / terminal and type: make all 

The .elf artifact can be used to:

- debug the firmware sensor;
- generate the flashable images such .bin or .hex

#advices:
Unfurtonately I received my board with only 2 days of contest reamainig, due this I have able time only to build the minimal functions such 
make the Flexio with OV7670 camera to work, and basics OS threads and basics image processing with gives if has flow and estimates the
percentage o fertilizer flow.

#License:
Oh Yes, this is a evolving project, so its has no license, which means you can use as your own purpose.

#Enjoy.


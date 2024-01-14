# Dex
A self balancing robot for the Rasberry Pi Pico
image here


## Starting Debugger
Plug in debugger and run:
```
openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -s tcl
```
confrim this starts on localhost:3333

open another terminal and run:
```
gdb-multiarch dex.elf
```
in this termincal type the command
```
target remote localhost:3333
```
load with 
```
load
```
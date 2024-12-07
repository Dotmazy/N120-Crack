# N120-Crack
In this repository there is all the source code i've made (bootloader) and used (rest) to crack the numwork n120.
If someone wanna help me cracking the n120 (if you're french it's even better because i'm french)

# DO NOT FOLLOW THIS TUTORIAL, IT'S WORK IN PROGRESS

# Here is the step to crack the n120 (WIP)

## Needed

Here is all the things you'll need:
- a raspberry pi (i've used a pi 4b but you can use any)
- a numwork N120 (look at the back of your calculator)
- 3 cables (to connect the raspberry to the numwork)

## Open the numwork and connections

First if you have any scripts or values don't forget to save it in your computer.
Next open the numwork (you'll probably need to buy a special screwdriver), be carefull and don't unplug the battery.

Then here is the connections:
|Numwork|Raspberry PI|
|:--|:--|
|VSS|GND|
|TX (PA9)|RX (GPIO 15)|
|RX (PA10)|TX (GPIO 14)|

![NUMWORK CONNECTIONS](https://github.com/user-attachments/assets/81d5159b-617a-40ef-8e0f-32e959350a6f)

## Flash the custom bootloader (unlocked)
Download the bootloader in **Release** Section.
## (WIP, please wait)

# Here is some information I can give you about the calculator

## Bottom connections
|Left|Right|
|--|--|
|RX|VREF+|
|TX|VBAT|
|3.3v|NRST|
|GND |-|
|-|-|

## Micro Processor Reference
STM32H725

## Screen Reference (not sure, taken from the n110)
ST7789V

# SSD1283A-raspberrypi
userspace SSD1283A sample code for raspberry pi 

## ref
https://github.com/ZinggJM/SSD1283A

https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md#troubleshooting

https://www.raspberrypi.org/documentation/hardware/raspberrypi/gpio/README.md

## hardware
the 130*130 SPI lcd module http://www.lcdwiki.com/1.6inch_SPI_Module_SSD1283A_SKU:MSP1601

## physical connection
cs,cd,reset,led -> 8, 27, 22, 17 
spi0

## lib used
see makefile

## start
rm *.o && make &&  ./exec

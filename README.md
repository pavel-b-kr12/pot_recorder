# pot_recorder
Record and play of potentiometer position and map  it to stepper motor position, LED dimmer or ws2812 

## Working:
1. press REC
2. move potentiometer or other input/sensor
3. stop recording. 
4. Play. 

## Features
- [x] record to SD card.
- [x] Several recordslots available. slot 9 is for settings of max Speed, Acceleration, cyclic replay.
- [x] steper motor output. Default is manual mode - move pot - motor meves according to it range
- [ ] TODO PWM (i.e. dimmer) output
- [ ] TODO WS2812b* output

- [x] support some use of US-015 or other ultrasonic 

## hardware
* ESP32 (support of atmega328 complately broken, if you want - i will fix it or upload last version with it working)
* SD cart (support of EEPROM probably complately broken, if you want - i will fix it)
* OLED IIC 128*32

## TODO
*upload video

## connection scheematics
![ESP32](ESP32-pin v0.51.png)





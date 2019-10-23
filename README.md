# poe_final_project

## Making dependecies work
To get dependicies working, just move the subfolders in your folder for Arduino libraries. If you want to get fancy with it, you can create soft links from your Arduino libraries folder to the dependencies in this repo. This makes it so that your arduino libraries folder can find these dependencies without having to move anything. You can do this as follows:

```
ln -s <your-arduino-libraries-dir>/Adafruit_9DOF Adafruit_9DOF
ln -s <your-arduino-libraries-dir>/Adafruit_L3GD20_U Adafruit_L3GD20_U
ln -s <your-arduino-libraries-dir>/Adafruit_LSM303DLHC Adafruit_LSM303DLHC
ln -s <your-arduino-libraries-dir>/Adafruit_Sensor Adafruit_Sensor
```

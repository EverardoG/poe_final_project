# poe_final_project

## Quick and Easy Setup

Typing these commands into your terminal will make it so that you have the repo and all the code you need on your computer.
```
git clone https://github.com/EverardoG/poe_final_project.git
git submodule init
git submodule update
```

To make your dependecies work, all you have to do is copy all of the folders inside of `dependencies` into wherever your `Arduino/libraries` folder is.

## Making dependecies work
Since all the dependencies are actually git submodules and submodules work weird, you're going to have to do some extra steps to get all the code you need. All you have to do is go into where you've cloned your repo and do the following:

<!---

```
git submodule init
git submodule update
```

To get dependicies working, just move the subfolders in your folder for Arduino libraries. If you want to get fancy with it, you can create soft links from your Arduino libraries folder to the dependencies in this repo. This makes it so that your arduino libraries folder can find these dependencies without having to move anything. You can do this as follows:

```
ln -s <your-arduino-libraries-dir>/Adafruit_9DOF Adafruit_9DOF
ln -s <your-arduino-libraries-dir>/Adafruit_L3GD20_U Adafruit_L3GD20_U
ln -s <your-arduino-libraries-dir>/Adafruit_LSM303DLHC Adafruit_LSM303DLHC
ln -s <your-arduino-libraries-dir>/Adafruit_Sensor Adafruit_Sensor
```
-->

# AI Foosball Table

## Michael Rencheck

## Dependencies
The following are other software packages required to run the software for the table:
  - ROS Melodic
  - Pololu tic Software
  - OpenCV

## Installation

Begin by installing some of the external software dependancies:
  - The instructions for installing ROS Melodic are found [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

  - The instructions to install the Pololu tic Software are found [here](https://github.com/pololu/pololu-tic-software). See relevant section in the BUILDING.md file to build the library from source, these were the [directions](https://github.com/pololu/pololu-tic-software/blob/master/BUILDING.md#building-from-source-on-linux-for-linux) I used.

After installing these create a new catkin workspace and clone this repository:

```
mkdir SinglePlayerFoosball
cd SinglePlayerFoosball
git clone https://github.com/rencheckyoself/single_player_foosball.git src
```

Then go into the repository and use wstool. This will download my forked version of the oCam package.
```
cd src
wstool update
```

Finally, go back to the top level of the directory and build the workspace:
```
cd ..
catkin_make
```

## Packages

### Control
`table_motor_control`: Files to interact with the TIC API

### Vision
This package is currently set up to follow the [ROS image_pipeline](http://wiki.ros.org/image_pipeline) structure. The `table_vision_sensing` package is set up to be camera agnostic, so feel free to use your own camera in place of the one linked in the parts list. The performance of the system may differ if your chosen camera runs at a slower FPS.

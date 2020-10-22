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

Then go into the repository and use wstool. This will download my forked version of the oCam package. It has been updated to function properly with ROS Melodic and adds some extra configuration for including the main launch file.
```
cd src
wstool update
```

Finally, go back to the top level of the directory and build the workspace:
```
cd ..
catkin_make
```

## Required Configuration

- Update `table_motor_control/config/motor_ids.yaml` with the serial numbers for your specific TIC boards. These can be obtained by using the ticgui or ticcli.

- Calibrate your camera. I have included the calibration file for my camera in the package files, but it may not be suitable for yours.

## Packages

### Control
This package provides an interface to work with the TIC Stepper Controller API through ROS by offering services.

Launch Files:
`tic_controllers.launch`: launch all of the nodes to interact with the motors

Nodes:
`tic_cmd`: main node that starts up the services to interact with each controller.

Configuration:
`motor_ids.yaml`: contains all of the identifying information for each controller. This file must be updated with the serial numbers for your purchased boards.

`*_settings.txt`: these files contain the detailed settings for each of the motors. They were initially generated by exporting the settings using the ticgui. These files can be used to update the settings of a given tic while everything is running using the `*_update_settings` service offered by the `tic_cmd` node.

### Vision
This package is currently set up to follow the [ROS image_pipeline](http://wiki.ros.org/image_pipeline) structure. The `table_vision_sensing` package is set up to be camera agnostic, so feel free to use your own camera in place of the one linked in the parts list. The performance of the system may differ if your chosen camera runs at a slower FPS.

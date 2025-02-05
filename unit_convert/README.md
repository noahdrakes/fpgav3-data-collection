# Data Conversion

This subfolder contains a Python script to convert bits (raw data) into SI unit values. Note that the units for torques are N*m, for positions are rad or m, and for velocities are rad/s or m/s.

## Preparation

Typically, the dVRK stack uses XML file to describe robot and store robot specific parameters, such as calibrated current or encoder offsets. Python, however, does a better job in handling json files. There is an existing application built upon Amp1394 library in the [mechatronics-software](https://github.com/jhu-cisst/mechatronics-software.git), called "sawRobotIO1394XMLtoJSON". You can compile the mechatronics-softwrae repository to get this application, or you can compile the dVRK ROS package to get it (this package contains the mechatronics-software, aka sawRobotIO, as a submodule). An example command for using this application to convert an XML file to json:

```
         sawRobotIO1394XMLtoJSON -c sawRobotIO1394-PSM1-26611.xml
```

## Running 

- Example command:

```
         python unit_convert.py -c sawRobotIO1394-PSM1-26611.xml.json -f capture_Mon_Dec__9_14_07_18_2024.csv
```

## Output

The program will output a csv file for each capture containing the following data for each axis:

*Timestamp,* *TorqueFeedback1*,..,*TorqueFeedbackN*, *TorqueCommand1*, *TorqueCommandN*, *PositionFeedback1*, *PositionFeedbackN*, *VelocityFeedback1*, *VelocityFeedbackN* in that order.

The filename for each capture is capture_[date and time]_unitConvert.csv

###### Contact Info
Send me an email if you have any questions.
Simon Hao
email: hao.yang@vanderbilt.edu

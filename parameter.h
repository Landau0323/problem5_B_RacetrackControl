//
// Created by 研究用 on 2017/09/05.
//

#ifndef PROBLEM5_B_RACETRACKCONTROL_PARAMETER_H
#define PROBLEM5_B_RACETRACKCONTROL_PARAMETER_H

const double PI=3.1415;

const double length=20.0;

const double speed=1.0;
const double max_steering_angle=PI/4.0;

//add drift parameter (offset for steering)
const double drift=0; //0 (P/PD) or 10*PI/18(PID);

const double tolerance=0.001;

const double radius=25.0;

#endif //PROBLEM5_B_RACETRACKCONTROL_PARAMETER_H

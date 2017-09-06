//
// Created by 研究用 on 2017/09/05.
//

#ifndef PROBLEM5_B_RACETRACKCONTROL_PARTICLE_H
#define PROBLEM5_B_RACETRACKCONTROL_PARTICLE_H

#include <map>
#include <string>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <random>

using namespace std;

class Particle {
    static int count_particle;

private:
    double x,y,theta;   //position and direction of the car
    double cre_previous,cre_integrate;
    int id;
    double tau,tau_derivative,tau_integrate;

public:
    Particle();

    void set_state(double,double,double);
    void move(double, double);
    void move_Pcontrol(double);
    void move_PDcontrol(double,double);
    void move_PIDcontrol(double,double,double);
    double crosstrack_error();

    void show_result();
    void write_result(string);

    double getX() const;
    double getY() const;
    double getTheta() const;
};


#endif //PROBLEM5_B_RACETRACKCONTROL_PARTICLE_H

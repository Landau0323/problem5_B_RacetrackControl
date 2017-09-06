//
// Created by 研究用 on 2017/09/05.
//

#include "Particle.h"
#include "parameter.h"

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

using namespace std;

int Particle::count_particle=0;

Particle::Particle() {
    //count and id of particle
    count_particle++;
    id=count_particle;
    tau=tau_derivative=tau_integrate=0;
}

void Particle::set_state(double input_x, double input_y, double input_theta) {
    x=input_x;
    y=input_y;
    theta=input_theta;

    cre_previous=0;
    cre_integrate=0;
}

void Particle::move(double input_alpha, double input_distance) {
    //drift
    input_alpha+=drift;

    //max steering angle
    if(input_alpha>max_steering_angle) input_alpha=max_steering_angle;
    else if(input_alpha<-max_steering_angle) input_alpha=-max_steering_angle;
    //positivity of distance
    if(input_distance<0) input_distance=0;

    double R=length/tan(input_alpha);   //R is the radius of the circle made by the rear tire
    double beta=input_distance/R;   //beta is the angle for the rear tire
    double cx=x-R*sin(theta);   //cx, cy are the position of the center of the circle made by the rear tire
    double cy=y+R*cos(theta);
//cout<<"steering="<<input_alpha<<" R="<<R<<" beta="<<beta<<endl;

    //if beta is not very small, we do the normal procedure
    if(abs(beta)>=tolerance) {
        x = cx + R * sin(theta + beta);
        y = cy - R * cos(theta + beta);
    }
        //if beta is very small, R diverges, so we regard that the particle is going straight.
    else{
        x+=input_distance*cos(theta);
        y+=input_distance*sin(theta);
    }
    theta += beta;

    //角度がmod 2piであることを実装
    if(theta<0) theta+=2.0*PI;
    else if(theta>2.0*PI) theta+= -2.0*PI;
}

void Particle::move_Pcontrol(double tau) {
    double steering=-tau*crosstrack_error();
    move(steering,speed);
    cout<<"steering="<<steering<<endl;
}

void Particle::move_PDcontrol(double tau,double tau_derivative) {
//show_result();
//cout<<"D term="<<-tau_derivative*(y-y_previous)<<endl;
//cout<<"P term="<<-tau*crosstrack_error()<<endl;
    double steering=-tau*crosstrack_error()
                    -tau_derivative*(crosstrack_error()-cre_previous);
    cre_previous=crosstrack_error();
    move(steering,speed);
    cout<<"steering="<<steering<<endl;
}

void Particle::move_PIDcontrol(double tau,double tau_derivative, double tau_integrate) {
    double steering=-tau*crosstrack_error()
                    -tau_derivative*(crosstrack_error()-cre_previous)
                    -tau_integrate*cre_integrate;
    cre_previous=crosstrack_error();
    cre_integrate+=crosstrack_error();
    move(steering,speed);
    cout<<"steering="<<steering<<" CTE="<<crosstrack_error()<<endl;
}

double Particle::crosstrack_error() {
        //first circle part
    if(x<radius)    return sqrt((x-radius)*(x-radius)+(y-radius)*(y-radius))-radius;
        //straight part
    else if(radius<=x && x<3*radius){
        //going left
        if(PI/2<theta && theta<PI*3/2) return -y;
        //going right
        else return y-2*radius;
    }
        //second circle part
    else if(3*radius<=x)    return sqrt((x-3*radius)*(x-3*radius)+(y-radius)*(y-radius))-radius;
}

void Particle::write_result(string filename) {
//    cout<<"writing data"<<endl;
    ofstream fout;
    fout.open(filename,ios::app);

    fout<< x<<" "<<y<<" "<<theta<<endl;
    fout.close();
}

void Particle::show_result() {
//    cout<<"id: "<<id<<endl;
    cout<<"[x="<<x<<" y="<<y<<" orient="<<theta<<"]"<<endl;
}

double Particle::getX() const {
    return x;
}
double Particle::getY() const {
    return y;
}
double Particle::getTheta() const {
    return theta;
}

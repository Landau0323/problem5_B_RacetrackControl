#include <iostream>
#include "Particle.h"
#include "parameter.h"

using namespace std;

int main() {
    //parameters in PID control
    double tau,tau_derivative,tau_integrate;
    tau=10.0;
    tau_derivative=15.0;
    tau_integrate=0;

    Particle car;
    //total distance: 4radius (straight) + 2*PI*radius (curves)=257
    int N=257;
    car.set_state(0,radius,PI/2);
    for(int i=0;i<N;i++){
        car.move_PIDcontrol(tau,tau_derivative,tau_integrate);
        car.show_result();
        car.write_result("car_track.dat");
    }

    return 0;
}
//
// Created by ajahueym on 15/08/20.
//

#ifndef DRONETEST_DRONEPARAMS_H
#define DRONETEST_DRONEPARAMS_H

/**
 * This class is used to describe the physical properties of a QuadCopter
 *
 * * */
struct DroneParams {
    double weight = 0; // kg
    double armLength = 0; // m
    double gravity = 9.81; // m/s²
    double inertiaX = 0; // kgm²
    double inertiaY = 0; // kgm²
    double inertiaZ = 0; // kgm²
    double thrustCoefficient = 0; // N/(rad²/s²)
    double torqueCoefficient = 0; // Nm/(rad²/s²)
};

#endif //DRONETEST_DRONEPARAMS_H

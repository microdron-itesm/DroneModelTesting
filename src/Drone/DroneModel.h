//
// Created by ajahueym on 15/08/20.
//

#ifndef DRONETEST_DRONEMODEL_H
#define DRONETEST_DRONEMODEL_H

#include <eigen3/Eigen/Dense>
#include "DroneParams.h"

/**
 * Based on "Embedded Flight Control Based on Adaptive Sliding Mode Strategy for a Quadrotor Micro Air Vehicle"
 * by Herman Castañeda and Jose Luis Gordillo.
 * https://www.mdpi.com/2079-9292/8/7/793
 * This class represents the mathematical model of a QuadCopter, its useful for simulating them and testing control systems.
 */
class DroneModel {
public:
    /**
     * Takes in a Drone Param to configure the drone's mathematical model
     * @param params
     */
    DroneModel(DroneParams params);

    /**
     * Receives the current time step in seconds
     * @param timeStep
     */
    void update(double timeStep);

    /**
     * Get the current rotors speed, unit is radians per second
     * @return
     */
    const std::vector<double> &getRotorSpeeds() const;

    /**
     * Set the current rotors speed, unit is radians per second
     * @return
     */
    void setRotorSpeeds(const std::vector<double> &rotorSpeeds);

    /**
     * Get the current configuration of the Drone Model
     * @return
     */
    const DroneParams &getParams() const;

    /**
     * Get the current position of the Drone in an Eigen Matrix, (x, y, z), unit is meters
     * @return
     */
    const Eigen::MatrixXd &getPosition() const;

    /**
     * Get the current orientation of the Drone in an Eigen Matrix, (roll, pitch, yaw), unit is radians
     * @return
     */
    const Eigen::MatrixXd &getOrientation() const;

private:
    static Eigen::MatrixXd positionRotationMatrix(const Eigen::MatrixXd &orientation);

    static Eigen::MatrixXd orientationRotationMatrix(const Eigen::MatrixXd &orientation);

    static Eigen::MatrixXd calculateThrustVector(double thrustCoefficient, const std::vector<double> &rotorSpeeds);

    static Eigen::MatrixXd calculateTorqueVector(double thrustCoefficient, double torqueCoefficient, double armLength,
                                                 const std::vector<double> &rotorSpeeds);

    DroneParams params;

    std::vector<double> rotorSpeeds{0, 0, 0, 0};
    Eigen::MatrixXd position{3, 1};
    Eigen::MatrixXd localLinearVelocity{3, 1};

    Eigen::MatrixXd orientation{3, 1}; // From -pi  to pi
    Eigen::MatrixXd localAngularVelocity{3, 1};
    Eigen::MatrixXd inertiaMatrix{3, 3}; // From -pi  to pi

    Eigen::MatrixXd globalGravityVector{3, 1};

};


#endif //DRONETEST_DRONEMODEL_H

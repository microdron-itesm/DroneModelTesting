//
// Created by ajahueym on 15/08/20.
//

#include "DroneModel.h"


DroneModel::DroneModel(DroneParams params) : params(params){
    inertiaMatrix <<    params.inertiaX,    0,                  0,
                        0,                  params.inertiaY,    0,
                        0,                  0,                  params.inertiaZ;

    localLinearVelocity << 0, 0, 0;
    localAngularVelocity << 0, 0, 0;
    orientation << 0, 0, 0;
    position << 0, 0 ,0;
    globalGravityVector << 0, 0, params.weight * params.gravity;

}

Eigen::MatrixXd DroneModel::positionRotationMatrix(const Eigen::MatrixXd& orientation){
    Eigen::MatrixXd rotationMatrix {3, 3};
    double cosRoll = std::cos(orientation(0));
    double cosPitch = std::cos(orientation(1));
    double cosYaw = std::cos(orientation(2));

    double sinRoll = std::sin(orientation(0));
    double sinPitch = std::sin(orientation(1));
    double sinYaw = std::sin(orientation(2));

    rotationMatrix <<   cosYaw * cosPitch,  -sinYaw * cosRoll + cosYaw * sinPitch * sinRoll,    sinYaw * sinRoll + cosYaw * sinPitch * cosRoll,
                        sinYaw * cosPitch,  cosYaw * cosRoll + sinYaw * sinPitch * sinRoll,     -cosYaw * sinRoll + sinYaw * sinPitch * cosRoll,
                        -sinPitch,          cosPitch * sinRoll,                                 cosPitch * cosRoll;
    return rotationMatrix;
}

Eigen::MatrixXd DroneModel::orientationRotationMatrix(const Eigen::MatrixXd& orientation){
    Eigen::MatrixXd rotationMatrix {3, 3};
    double cosRoll = std::cos(orientation(0));
    double cosPitch = std::cos(orientation(1));

    double sinRoll = std::sin(orientation(0));
    double tanPitch = std::tan(orientation(1));

    rotationMatrix <<   1,  sinRoll * tanPitch,     cosRoll * tanPitch,
                        0,  cosRoll,                -sinRoll,
                        0,  sinRoll / cosPitch,     cosRoll / cosPitch;

    return rotationMatrix;
}
Eigen::MatrixXd DroneModel::calculateThrustVector(double thrustCoefficient, const std::vector<double>& rotorSpeeds){
    Eigen::MatrixXd result{3, 1};
    result << 0, 0, -(thrustCoefficient * std::pow(rotorSpeeds[0], 2) + thrustCoefficient * std::pow(rotorSpeeds[1], 2) + thrustCoefficient * std::pow(rotorSpeeds[2], 2) + thrustCoefficient * std::pow(rotorSpeeds[3], 2));
    return result;
}

Eigen::MatrixXd DroneModel::calculateTorqueVector(double thrustCoefficient, double torqueCoefficient, double armLength, const std::vector<double>& rotorSpeeds){
    Eigen::MatrixXd constantsMatrix{3, 4};
    Eigen::MatrixXd rotorSpeedsMatrix{4, 1};
    double thrustToArmLengthRatio = (armLength * thrustCoefficient) / std::sqrt(2);
    constantsMatrix <<  -thrustToArmLengthRatio,    thrustToArmLengthRatio,    thrustToArmLengthRatio,     -thrustToArmLengthRatio,
                        thrustToArmLengthRatio,     thrustToArmLengthRatio,     -thrustToArmLengthRatio,    -thrustToArmLengthRatio,
                        -torqueCoefficient,         torqueCoefficient,          -torqueCoefficient,         torqueCoefficient;

    rotorSpeedsMatrix << std::pow(rotorSpeeds[0], 2), std::pow(rotorSpeeds[1], 2),std::pow(rotorSpeeds[2], 2), std::pow(rotorSpeeds[3], 2);

    return constantsMatrix * rotorSpeedsMatrix;
}

void DroneModel::update(double timeStep) {

    Eigen::MatrixXd gravityVector = positionRotationMatrix(orientation).inverse() * globalGravityVector;
    Eigen::MatrixXd torqueVector = calculateTorqueVector(params.thrustCoefficient, params.torqueCoefficient, params.armLength, rotorSpeeds);

    Eigen::MatrixXd thrustVector = calculateThrustVector(params.thrustCoefficient, rotorSpeeds);

    Eigen::MatrixXd forceVector = gravityVector + thrustVector;

    Eigen::MatrixXd angularAcceleration = inertiaMatrix.inverse() * torqueVector;
    localAngularVelocity += angularAcceleration * timeStep;

    Eigen::MatrixXd linearAcceleration = forceVector / params.weight;
    localLinearVelocity += linearAcceleration * timeStep;

    Eigen::MatrixXd globalLinearVelocity = positionRotationMatrix(orientation) * localLinearVelocity;
    Eigen::MatrixXd globalAngularVelocity = orientationRotationMatrix(orientation).inverse() * localAngularVelocity;

    /**
     * Make it so  the reference frame follows right hand rule, on paper its upside down
     */
    globalLinearVelocity(1) *= -1;
    globalLinearVelocity(2) *= -1;

    position += globalLinearVelocity * timeStep;
    orientation += globalAngularVelocity * timeStep;
}

const std::vector<double> &DroneModel::getRotorSpeeds() const {
    return rotorSpeeds;
}

void DroneModel::setRotorSpeeds(const std::vector<double> &rotorSpeeds) {
    DroneModel::rotorSpeeds = rotorSpeeds;
}

const DroneParams &DroneModel::getParams() const {
    return params;
}

const Eigen::MatrixXd &DroneModel::getPosition() const {
    return position;
}

const Eigen::MatrixXd &DroneModel::getOrientation() const {
    return orientation;
}


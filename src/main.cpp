#include <iostream>
#include <thread>
#include "Drone/DroneModel.h"
#include <cmath>

int main() {
    DroneParams rollingSpiderParams;
    rollingSpiderParams.weight = 0.068;
    rollingSpiderParams.armLength = 0.062;
    rollingSpiderParams.inertiaX = 6.86 * std::pow(10, -5);
    rollingSpiderParams.inertiaY = 9.2 * std::pow(10, -5);
    rollingSpiderParams.inertiaZ = 1.366 * std::pow(10, -4);
    rollingSpiderParams.thrustCoefficient = 2.64 * std::pow(10, -8);
    rollingSpiderParams.torqueCoefficient = 7.8263 * std::pow(10, -4);
    DroneModel rollingSpider(rollingSpiderParams);

    bool isRunning = true;
    auto lastUpdate = std::chrono::high_resolution_clock::now();
    while (isRunning) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        double timeStep = std::chrono::duration<double>(currentTime - lastUpdate).count();
        lastUpdate = std::chrono::high_resolution_clock::now();
        rollingSpider.update(timeStep);
        rollingSpider.setRotorSpeeds({2515, 2515, 2515, 2515});
        std::cout << "Drone z: " << rollingSpider.getPosition()(2) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    return 0;
}

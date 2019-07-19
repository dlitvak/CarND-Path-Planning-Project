//
// Created by Dimitriy Litvak on 2019-06-12.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <map>
#include "helpers.h"
#include "Road.h"

class Car {

public:
    Car();
    Car(int id, double x, double y, double vx, double vy, double s, double d, Road *r);
    explicit Car(Car *cpCar);

    double getX() const {
        return x;
    }

    double getY() const{
        return y;
    }

    double getVx() const {
        return vx;
    }

    double getVy() const {
        return vy;
    }

    double getS() const {
        return s;
    }

    double getD() const {
        return d;
    }

    Road::LANE getLane() const {
        return lane;
    }

    int getId() const {
        return id;
    }

    void setId(int i) {
        this->id = i;
    }

    double getYaw() const {
        return this->yaw;
    }

    double getVel() const{
        return this->vel;
    }

    Road* getRoad() const {
        return road;
    }

    Car position_at(double t);

protected:
    int id = -1;  // no-car id
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;

    Road::LANE lane = Road::INVALID_LANE;
    double vel;
    double yaw;

    Road *road = nullptr;

};


#endif //PATH_PLANNING_CAR_H

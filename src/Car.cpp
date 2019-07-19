//
// Created by Dimitriy Litvak on 2019-06-12.
//

#include "Car.h"

using std::cout;
using std::endl;

Car::Car(int id, double x, double y, double vx, double vy, double s, double d, Road *r) : Car() {
    this->id = id;
    this->x =  x;
    this->y =  y;
    this->vx = vx;
    this->vy = vy;
    this->s =  s;
    this->d =  d;

    switch ((int)(d/Road::LANE_WIDTH)) {
        case 0:
            this->lane = Road::LEFT_LANE;
            break;
        case 1:
            this->lane = Road::MIDDLE_LANE;
            break;
        case 2:
            this->lane = Road::RIGHT_LANE;
            break;
    }

    this->vel = sqrt(vx*vx + vy*vy);
    this->yaw = atan2(vy, vx);

    this->road = r;
}

Car::Car() {
    this->id = -1;
    this->x =  0;
    this->y =  0;
    this->vx = 0;
    this->vy = 0;
    this->s =  0;
    this->d =  0;

    this->vel = 0;
    this->yaw = 0;
}

Car::Car(Car *cpCar) : Car() {
    this->id = cpCar->getId();
    this->lane = cpCar->getLane();
    this->x = cpCar->getX();
    this->y = cpCar->getY();
    this->vx = cpCar->getVx();
    this->vy = cpCar->getVy();
    this->vel = cpCar->getVel();
    this->s = cpCar->getS();
    this->d = cpCar->getD();

    this->yaw = cpCar->getYaw();
    this->road = cpCar->getRoad();
}

Car Car::position_at(double t) {
    Car pred;

    pred.id = this->id;

    pred.x =  this->x + this->vx*t;
    pred.y =  this->y + this->vy*t;

    assert(this->road != nullptr);
    Road::Map map = this->road->getMap();
    vector<double> fren = getFrenet(pred.x, pred.y, this->yaw, map.map_waypoints_x, map.map_waypoints_y);
    pred.s =  fren[0];
    pred.d =  fren[1];

    pred.lane = (Road::LANE)(pred.d/Road::LANE_WIDTH);

    //cout << "id: " << id << ", ax: " << ax << ", ay: " << ay << ", as: " << as << ", ad: " << ad << ", pred.x: " << pred.x << ", pred.y: " << pred.y << ", pred.s: " << pred.s << ", pred.d: " << pred.d << ", pred.vel: " << pred.vel << ", pred.lane: " << pred.lane  << endl;

    return pred;
}










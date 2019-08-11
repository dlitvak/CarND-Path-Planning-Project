//
// Created by Dimitriy Litvak on 2019-06-12.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <map>
#include "spline.h"
#include "helpers.h"
#include "Road.h"

class Car {

public:
    constexpr static double SPLINE_POINT_1 = 30;
    constexpr static double SPLINE_POINT_2 = 90;
    constexpr static double SPLINE_POINT_3 = 120;

    constexpr static double DT = 0.02;                  // sample time, sec
    constexpr static double PREDICTION_FAR_HORIZON_SEC = 5.;
    constexpr static double PREDICTION_FAR_HORIZON_TICKS = PREDICTION_FAR_HORIZON_SEC/DT;

    Car();
    Car(int id, double x, double y, double vx, double vy, double s, double d, Road *r);
    explicit Car(Car *cpCar);

    struct Position {
        double pt_x = 0;
        double pt_y = 0;
        double pt_s = 0;
        double pt_d = 0;
    };

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

    vector<Position> getPrediction() const {
        return prediction;
    }

    Position position_at(double t_sec);

    bool operator<(const Car &c2) const;

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

    vector<Position> getPath(const vector<double> &previous_path_x, const vector<double> &previous_path_y, double target_spline1_s,
                             double target_spline1_d, double target_v, double prediction_time_sec);

private:
    vector<Position> prediction;

};


#endif //PATH_PLANNING_CAR_H

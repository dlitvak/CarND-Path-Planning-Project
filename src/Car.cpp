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

    this->prediction = this->getPath({}, {}, SPLINE_POINT_1, this->d, this->vel, Car::PREDICTION_FAR_HORIZON_SEC);
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

    this->prediction = cpCar->getPrediction();
}

/**
 * This is function creates a very rough estimate of the car position in the future.
 *
 * @param t_sec time at which the car's position is desired
 * @return
*/
Car::Position Car::position_at(double t_sec) {
    assert(t_sec <= Car::PREDICTION_FAR_HORIZON_SEC);

    int tick = int(t_sec / DT);

    //TODO rem cout << "id: " << id << ", ax: " << ax << ", ay: " << ay << ", as: " << as << ", ad: " << ad << ", pred.x: " << pred.x << ", pred.y: " << pred.y << ", pred.s: " << pred.s << ", pred.d: " << pred.d << ", pred.vel: " << pred.vel << ", pred.lane: " << pred.lane  << endl;

    return this->prediction[tick];
}

// get the path for a remote s,d coord with a target velocity
vector<Car::Position> Car::getPath(const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                   const double target_spline1_s, const double target_spline1_d, const double target_v,
                                   const double prediction_time_sec) {
    assert(target_spline1_s >= SPLINE_POINT_1 && target_spline1_s < SPLINE_POINT_2);

    vector<double> ptsx;
    vector<double> ptsy;

    int prev_size = previous_path_x.size();
    double ref_x = this->x;
    double ref_y = this->y;
    double ref_s = this->s;
    double ref_yaw = this->yaw;
    double prev_ref_x;
    double prev_ref_y;

    if (prev_size < 2) {
        prev_ref_x = ref_x - cos(ref_yaw);
        prev_ref_y = ref_y - sin(ref_yaw);
    } else {
        prev_ref_x = previous_path_x[prev_size - 2];
        prev_ref_y = previous_path_y[prev_size - 2];

        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
    }

    //std::cout << "prev_sz: " << prev_size << std::endl;
    ptsx.push_back(prev_ref_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_ref_y);
    ptsy.push_back(ref_y);

    Road::Map map = this->road->getMap();
    vector<double> next_wp0 = getXY(addFrenetCircSDistance(ref_s, target_spline1_s), target_spline1_d, map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);
    vector<double> next_wp1 = getXY(addFrenetCircSDistance(ref_s, SPLINE_POINT_2), target_spline1_d, map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);
    vector<double> next_wp2 = getXY(addFrenetCircSDistance(ref_s, SPLINE_POINT_3), target_spline1_d, map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // transform into car's coord system
    for (u_long p = 0; p < ptsx.size(); ++p) {
        double shift_x = ptsx[p] - ref_x;
        double shift_y = ptsy[p] - ref_y;

        //cout << "x,y: " << ptsx[p] << "," << ptsy[p] << "; shift: " << shift_x << "," << shift_y << "; yaw, cos, sin: " << ref_yaw << "," << cos(ref_yaw)<< "," << sin(ref_yaw) << ", s: " << car_s << endl;

        ptsx[p] = shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw);
        ptsy[p] = shift_y * cos(ref_yaw) - shift_x * sin(ref_yaw);
    }

    // add points to the vector returned to the sim
    vector<Position> path;

    // add previous path points to the next set, so that the total path is continuous
    for (int p = 0; p < prev_size; ++p) {
        Position pos;
        double px = previous_path_x[p];
        double py = previous_path_y[p];
        pos.pt_x = px;
        pos.pt_y = py;

        vector<double> fren = getFrenet(px, py, ref_yaw, map.map_waypoints_x, map.map_waypoints_y);
        pos.pt_s = fren[0];
        pos.pt_d = fren[1];

        path.push_back(pos);
    }

    tk::spline spl;
    spl.set_points(ptsx, ptsy);

    double target_x = SPLINE_POINT_1;
    double target_y = spl(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    assert(target_v >= 0);

    double tV = target_v;
    if (target_v == 0)
        tV = 0.000001;  //prevent division by zero
    double inc = target_x / (target_dist / (tV * DT));

    //TODO rem cout << "traj_vel: " << (traj.v >= 0 ? std::to_string(traj.v) : "none") << ", tar_vel_after: " << this->targetVel << endl;

    const u_int PREDICTION_TICKS = int(prediction_time_sec / DT);
    assert(PREDICTION_TICKS > prev_size);
    for (int p = 1; p <= PREDICTION_TICKS - prev_size; ++p) {
        double x_pnt = inc * p;
        double y_pnt = spl(x_pnt);

        // transform back from car's coord system into global map coord
        double rot_x = x_pnt * cos(ref_yaw) - y_pnt * sin(ref_yaw);
        double rot_y = y_pnt * cos(ref_yaw) + x_pnt * sin(ref_yaw);

        double px = rot_x + ref_x;
        double py = rot_y + ref_y;
        Position pos;
        pos.pt_x = px;
        pos.pt_y = py;

        vector<double> fren = getFrenet(px, py, ref_yaw, map.map_waypoints_x, map.map_waypoints_y);
        pos.pt_s = fren[0];
        pos.pt_d = fren[1];

        path.push_back(pos);
    }

    return path;
}

bool Car::operator<(const Car &c2) const {
    return frenetCircSDistanceBetweenCars(this->s, c2.getS()) < 0;
}











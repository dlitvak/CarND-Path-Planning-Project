//
// Created by Dimitriy Litvak on 2019-06-12.
//

#ifndef PATH_PLANNING_EGO_H
#define PATH_PLANNING_EGO_H

#include "Car.h"
#include "spline.h"
#include "helpers.h"
#include "json.hpp"
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <ctime>

using std::vector;
using std::map;
using std::string;

//TODO create internal Planner class
class Ego : virtual public Car {
public:
    constexpr static double MAX_ACCEL = 9.99;              // m/s**2
    constexpr static double MAX_JERK = 9.99;               // m/s**3
    constexpr static double MAX_TIME_OUT_OF_LANE = 2.99;    // sec

    constexpr static double PREDICTION_HORIZON = 50;
    // larger PATH_HORIZON results in Ego's slower reaction time.
    // Ego follows the old PATH_HORIZON points before it switches to the new target points.
    // PATH_HORIZON should be less than PREDICTION_HORIZON
    constexpr static double PATH_HORIZON = 30;

    // Distance at which the  other cars become visible to radar.
    constexpr static double OTHER_CAR_DETECTION_HORIZON = 60;

    constexpr static double SPLINE_POINT_1 = 30;
    constexpr static double SPLINE_POINT_2 = 60;
    constexpr static double SPLINE_POINT_3 = 90;

    constexpr static double DT = 0.02;                  // sample time, sec

    constexpr static double PREFERRED_VEHICLE_FOLLOWING_DISTANCE = 30;
    constexpr static double MIN_VEHICLE_FOLLOWING_DISTANCE = 20;    // this should leave about 6m between bumpers
    constexpr static double IN_LANE_COLLISION_DISTANCE = 4.5;    // dist b/w cars centers when collision happens
    constexpr static double SIDE_COLLISION_DISTANCE = 2.5;    // dist b/w cars centers when collision happens

    constexpr static double LANE_CHANGE_TRIGGER_VELOCITY_DIFF_MS = 1;    //

    explicit Ego(Road *road);

    // INTERNAL DATA TYPES

    struct Path {
        vector<double> pts_x;
        vector<double> pts_y;
        vector<double> pts_s;
        vector<double> pts_d;
    };

    enum class FSM_STATES : int {
        CS, KL, PLCL, PLCR, LCL, LCR
    };

    class Trajectory {
    public:
        Trajectory() = default;

        Trajectory(FSM_STATES st, Road::LANE targLane, double s, double d, double v) {
            this->state = st;
            this->s = s;
            this->d = d;
            this->v = v;
            this->target_lane = targLane;
        }

        FSM_STATES state = FSM_STATES::CS;
        Road::LANE target_lane = Road::LANE::INVALID_LANE;
        double s = -1;
        double d = -1;
        double v = -1;
    };

    // GETTERS AND SETTERS
    void setX(double x) {
        this->x = x;
    }

    void setY(double y) {
        this->y = y;
    }

    void setVx(double vx) {
        this->vx = vx;
    }

    void setVy(double vy) {
        this->vy = vy;
    }

    void setS(double s) {
        this->s = s;
    }

    void setD(double d) {
        this->d = d;
    }

    Ego::FSM_STATES &getFsmState() {
        return this->fsm_state;
    }

    void setFsmState(const FSM_STATES &fsmState) {
        this->fsm_state = fsmState;
    }

    void setLane(Road::LANE lane) {
        this->lane = lane;
    }

    void setTelemetry(
        nlohmann::basic_json<map, vector, string, bool, int64_t, uint64_t, double, std::allocator, nlohmann::adl_serializer> &json);

    //MEMBER FUNCTIONS
    // Get the path for Ego according to the planned trajectory
    Path get_best_path();

private:
    // Note: this->targetVel is the desired current speed (target speed).  Telemetry vel. is due to the previous points that the car tries to follow.
    double targetVel;

    // Previous path data
    vector<double> previous_path_x;
    vector<double> previous_path_y;

    //Telemetry data
    double end_path_s = 0;
    double end_path_d = 0;

    // Logging params
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
    int roadLap = 1;
    bool roadLapUpdated = false;

    map<int, Car> other_cars;

    Car cars_behind[Road::NUM_LANES];
    Car cars_ahead[Road::NUM_LANES];

    Road* road;
    double lane_speeds[Road::NUM_LANES] = {0.,  0., 0.};

    FSM_STATES fsm_state = FSM_STATES::CS;

    // has the lane been changed/crossed while in LC state
    bool laneChangeFinished = false;

    // MONITOR ACCEL and JERK
    double prev_vel = 0;
    double prev_accel = INT_MAX;
    std::chrono::steady_clock::time_point prev_t;
    bool prevTSet = false;

    Path getPath(const Ego::Trajectory &traj);

    vector<Ego::Trajectory> gen_keep_lane_trajectory();

    vector<Ego::Trajectory> gen_lane_change_trajectories(FSM_STATES state);

    vector<Ego::Trajectory> gen_prep_lane_change_trajectory(FSM_STATES state);

    vector<double> get_kinematics(Road::LANE ln);

    // Obtain all physically possible successor FSM states
    vector<FSM_STATES> get_possible_successor_states();

    static vector<Car> generate_prediction(Car &car);

    vector<Ego::Trajectory> generate_trajectory(FSM_STATES next_state);

    bool get_vehicle_behind(Road::LANE ln, Car &rVehicle);

    bool get_vehicle_ahead(Road::LANE ln, Car &rVehicle);

    void calculate_lane_speeds();

    void set_cars_ahead_and_behind();

    static double frenetCircSDistanceBetweenCars(double car1, double car2);

    void recordIncident();

    // COST CALCULATION

    double calculate_traj_path_cost(const Trajectory &trajectory, map<int, vector<Car>> &other_car_predictions, Path &traj_path);

    double inefficiency_cost(const Ego::Trajectory &trajectory, const Ego::Path &traj_path,
                             const map<int, vector<Car>> &other_car_predictions);

    double collision_and_proximity_cost(const Ego::Trajectory &trajectory, const Ego::Path &ego_path,
                                        const map<int, vector<Car>> &other_car_predictions);

    double lane_change_readiness_cost(const Trajectory &trajectory, const Path &traj_path, const map<int, vector<Car>> &other_car_predictions);

    double off_center_lane_cost(const Trajectory &trajectory, const Path &traj_path, const map<int, vector<Car>> &other_car_predictions);



    enum COST_TYPES {
        COLLISION_COST, INEFFICIENCY_COST, PREPARE_LANE_CHANGE_COST
    };

    const map<Ego::FSM_STATES, map<COST_TYPES, double>> COST_WEIGHT_LIST =
        {{Ego::FSM_STATES::KL, {{COLLISION_COST, 5}, {INEFFICIENCY_COST, 3}}},
         {Ego::FSM_STATES::PLCL, {{COLLISION_COST, 10}, {INEFFICIENCY_COST, 3}, {PREPARE_LANE_CHANGE_COST, 1}}},
         {Ego::FSM_STATES::PLCR, {{COLLISION_COST, 10}, {INEFFICIENCY_COST, 3}, {PREPARE_LANE_CHANGE_COST, 1}}},
         {Ego::FSM_STATES::LCL, {{COLLISION_COST, 10}, {INEFFICIENCY_COST, 3}}},
         {Ego::FSM_STATES::LCR, {{COLLISION_COST, 10}, {INEFFICIENCY_COST, 3}}}
        };
    const map<COST_TYPES, std::function<double(Ego&, const Trajectory &, const Path &,
                                               const map<int, vector<Car>> &)
    >> COST_FUNC_MAP = {{COLLISION_COST, &Ego::collision_and_proximity_cost}, {INEFFICIENCY_COST, &Ego::inefficiency_cost}, {PREPARE_LANE_CHANGE_COST, &Ego::lane_change_readiness_cost}};

    double getEstimatedAccel() const;
};


#endif //PATH_PLANNING_EGO_H

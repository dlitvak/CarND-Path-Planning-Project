//
// Created by Dimitriy Litvak on 2019-06-12.
//

#ifndef PATH_PLANNING_EGO_H
#define PATH_PLANNING_EGO_H

#include "Car.h"
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
using std::cout;
using std::cerr;
using std::endl;

class Ego : virtual public Car {
public:
    constexpr static double MAX_ACCEL = 9.5;              // m/s**2
    constexpr static double MAX_JERK = 9.99;               // m/s**3
    constexpr static double MAX_TIME_OUT_OF_LANE = 3.0;    // sec

    constexpr static double PREDICTION_NEAR_HORIZON_SEC = 1.;
    constexpr static int PREDICTION_NEAR_HORIZON_TICKS = int(PREDICTION_NEAR_HORIZON_SEC / DT);

    // Ego follows ALL PATH_HORIZON_TICKS points before it switches to the new target points.
    // Larger PATH_HORIZON_TICKS results in Ego's slower reaction time.
    // PATH_HORIZON_TICKS should always be less than PREDICTION_NEAR_HORIZON_TICKS and, preferably, < (SPLINE_POINT_1/SPEED_LIMIT_MPS)/DT
    constexpr static double PATH_HORIZON_SEC = 0.5;
    constexpr static int PATH_HORIZON_TICKS = int(PATH_HORIZON_SEC / DT);

    // Distance at which the  other cars become visible to radar.
    constexpr static double OTHER_CAR_DETECTION_HORIZON = 60;

    constexpr static double PREFERRED_VEHICLE_FOLLOWING_DISTANCE = 30;  // leaves about 26m between bumpers
    constexpr static double MIN_VEHICLE_FOLLOWING_DISTANCE = 10;    // this should leave about 6m between bumpers
    constexpr static double IN_LANE_COLLISION_DISTANCE = 4.5;    // dist b/w cars centers when collision happens: de facto car length
    constexpr static double SIDE_COLLISION_DISTANCE = 2.5;    // dist b/w cars centers when collision happens: de facto car width

    constexpr static double LANE_CHANGE_TRIGGER_VELOCITY_DIFF_MS = 1;    //

    constexpr static long MIN_KL_TIME_AFTER_LC_MILLIS = 5000;   // After switching from LC, stay in KL state these many
                                                                // milliseconds unless there's a collision.

    // "Out of lane time" is when Ego touches the lane dividing line.  Here, I'm approximating the maximum total lane
    // change time from the middle of a start lane to the middle of the target lane, assuming constant change lane velocity.
    constexpr static double MAX_TOTAL_LANE_CHANGE_TIME_SEC =  Road::LANE_WIDTH * MAX_TIME_OUT_OF_LANE/SIDE_COLLISION_DISTANCE;

    explicit Ego(Road *road);

    // INTERNAL DATA TYPES

    enum class FSM_STATES : int {
        CS, KL, PLCL, PLCR, LCL, LCR
    };

    class Trajectory {
    public:
        Trajectory() = default;

        Trajectory(FSM_STATES st, Road::LANE targLane, double s, double d, double v) {
            this->target_state = st;
            this->target_spline1_s = s;
            this->target_d = d;
            this->target_v = v;
            this->target_lane = targLane;
        }

        FSM_STATES target_state = FSM_STATES::CS;
        Road::LANE target_lane = Road::LANE::INVALID_LANE;
        double target_spline1_s = -1;
        double target_d = -1;
        double target_v = -1;

        int potential_collision_car_id = -1;

        vector<Car::Position> path;
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
    vector<Car::Position> get_best_path();

    bool isInLaneChangeState();
    bool isInPrepLaneChangeState();

private:
    // Previous path data
    vector<double> previous_path_x;
    vector<double> previous_path_y;

    //Telemetry data
    double end_path_s = 0;
    double end_path_d = 0;

    // Logging params
    std::chrono::system_clock::time_point startTime;
    bool timerStarted = false;
    int roadLap = 1;
    bool roadLapUpdated = false;

    map<int, Car> other_cars_id_map;
    map<Road::LANE, vector<Car>> other_cars_lane_map;

    Car cars_behind[Road::NUM_LANES];
    Car cars_ahead[Road::NUM_LANES];

    double lane_speeds[Road::NUM_LANES] = {0.,  0., 0.};

    FSM_STATES fsm_state = FSM_STATES::CS;

    // has the lane been changed/crossed while in LC state
    bool laneCrossedInLCState = false;

    // KL state timer to prevent excessive lane weaving.  After switching from LC state, stay in KL state for some seconds
    // unless there's a collision.  This is managed by weights*cost.
    std::chrono::high_resolution_clock::time_point KL_state_start_timepoint;

    vector<Ego::Trajectory> gen_keep_lane_trajectory();

    vector<Ego::Trajectory> gen_lane_change_trajectories(FSM_STATES state);

    vector<Ego::Trajectory> gen_prep_lane_change_trajectory(FSM_STATES state);

    double get_kinematics(Road::LANE ln);

    // Obtain all physically possible successor FSM states
    vector<FSM_STATES> get_possible_successor_states();

    vector<Ego::Trajectory> generate_trajectory(FSM_STATES next_state);

    bool get_vehicle_behind(Road::LANE ln, Car &rVehicle);

    bool get_vehicle_ahead(Road::LANE ln, Car &rVehicle);

    void calculate_lane_speeds();

    void set_cars_ahead_and_behind();

    static double calculateCurveRadius( double x0, double y0, double x1, double y1, double x2, double y2);

    static bool isInLaneChangeState(FSM_STATES st);
    static bool isInPrepLaneChangeState(FSM_STATES st);

    static double estimate_curve_radius(Car::Position p1, Car::Position p2, Car::Position p3);

    static double get_max_tot_accel(const Trajectory &trajectory);

    double get_out_of_lane_time(const Trajectory &trajectory);

    void recordIncident();

    // COST CALCULATION

    double calculate_traj_path_cost(Trajectory &trajectory);

    double inefficiency_cost(const Ego::Trajectory &trajectory);

    double collision_and_proximity_cost(Ego::Trajectory &trajectory);

    double lane_change_readiness_cost(const Trajectory &trajectory);

    double lane_off_center_cost(const Trajectory &trajectory);

    double lane_weaving_cost(const Trajectory &trajectory);

    enum COST_TYPES {
        COLLISION_COST, INEFFICIENCY_COST, PREPARE_LANE_CHANGE_COST, OFF_LANE_CENTER_COST, LANE_WEAVING_COST
    };

    // MIN_COLLISION_COST should be the highest cost; it should be higher than the sum of all non-colln costs in every state.
    // PLC and LC states should have higher collision cost than KL, so that when collision/proximity cost is significant,
    // KL should be the highly preferred state.  I had to decide on a MIN_COLLISION_COST multiplication coeff. for PLC/LC states by trial.
    const int MIN_COLLISION_COST = 9;
    const double LC_COLLN_COEFF = 2;
    const map<Ego::FSM_STATES, map<COST_TYPES, double>> COST_WEIGHT_MAP =
        {{Ego::FSM_STATES::KL, {{COLLISION_COST, MIN_COLLISION_COST}, {INEFFICIENCY_COST, 3}}},
         {Ego::FSM_STATES::PLCL, {{COLLISION_COST, LC_COLLN_COEFF*MIN_COLLISION_COST}, {INEFFICIENCY_COST, 3}, {PREPARE_LANE_CHANGE_COST,1},   {OFF_LANE_CENTER_COST,  2}, {LANE_WEAVING_COST, 4}}},
         {Ego::FSM_STATES::PLCR, {{COLLISION_COST, LC_COLLN_COEFF*MIN_COLLISION_COST}, {INEFFICIENCY_COST, 3}, {PREPARE_LANE_CHANGE_COST,1},   {OFF_LANE_CENTER_COST,  2}, {LANE_WEAVING_COST, 4}}},
         {Ego::FSM_STATES::LCL, {{COLLISION_COST, LC_COLLN_COEFF*MIN_COLLISION_COST}, {INEFFICIENCY_COST, 3}}},
         {Ego::FSM_STATES::LCR, {{COLLISION_COST, LC_COLLN_COEFF*MIN_COLLISION_COST}, {INEFFICIENCY_COST, 3}}}
        };
    const map<COST_TYPES, std::function<double(Ego&, Trajectory &)
    >> COST_FUNC_MAP = {{COLLISION_COST,                  &Ego::collision_and_proximity_cost},
                        {INEFFICIENCY_COST,               &Ego::inefficiency_cost},
                        {PREPARE_LANE_CHANGE_COST,        &Ego::lane_change_readiness_cost},
                        {OFF_LANE_CENTER_COST,            &Ego::lane_off_center_cost},
                        {LANE_WEAVING_COST,               &Ego::lane_weaving_cost}};

    // Target Vel CONTROLLER.  New controller class instantiated with each tick.
    // Allows setting acceleration only once per tick.
    // this->targetVel is the desired current speed (target speed).  Telemetry vel. is the current car vel.
    class TargetV_Controller {
    public:
        enum class ACCEL_DIR : short { PLUS = 1, MINUS = -1};

        TargetV_Controller() = default;

        TargetV_Controller(vector<double> &px, vector<double> &py, double tv) {
            assert(px.size() == py.size());

            this->targetVel = tv;

            // keep track of the normal acceleration at all times to limit the tangential acceleration and
            // prevent the total acceleration from exceeding the MAX_ACCEL.
            int path_size = px.size();
            if (path_size >= 3) {
                Car::Position p1, p2, p3;
                p1.pt_x = px[path_size-1];
                p1.pt_y = py[path_size-1];
                p2.pt_x = px[path_size-2];
                p2.pt_y = py[path_size-2];
                p3.pt_x = px[path_size-3];
                p3.pt_y = py[path_size-3];

                double R =  Ego::estimate_curve_radius(p1, p2, p3);
                this->accelN = this->targetVel * this->targetVel / R;
            }
        }

        bool accelerate(double acc) {
            assert(count_set_vel == 0);
            count_set_vel++;    // lock against multiple accidental targetVel updates

            this->targetVel = getNewVelocity(acc);
            return true;
        }

        bool accelerate_max(ACCEL_DIR acc_dir) {
            return accelerate((short)acc_dir * this->getAbsMaxAccelT());
        }

        double getNewVelocity(double acc) {
            double abs_acc = abs(acc);

            ACCEL_DIR dir = acc < 0 ? ACCEL_DIR::MINUS : ACCEL_DIR::PLUS;
            acc = (short)dir * std::min(abs_acc, this->getAbsMaxAccelT());

            double new_vel = this->targetVel + acc * DT;
            if (new_vel > Road::SPEED_LIMIT_MPS)
                new_vel = Road::SPEED_LIMIT_MPS;
            else if (new_vel < 0)
                new_vel = 0;

            return new_vel;
        }

        double getMaxNewVelocity(ACCEL_DIR acc_dir) {
            return this->getNewVelocity((short)acc_dir * this->getAbsMaxAccelT());
        }

        // Transfer of control to the driver warning
        static void emergency(const string &msg) {
            cout << "##################################################################################################################" << endl;
            cout << "# EMERGENCY : Running out of options to avoid a collision. Driver may need to take control!" << endl;
            cout << "# " << msg << endl;
            cout << "##################################################################################################################" << endl;
        }

        void setTargetVel(double tv) {
            assert(count_set_vel == 0);
            count_set_vel++;    // lock against multiple accidental targetVel updates

            assert(tv >= 0);  // illegal to drive bkwds
            assert(this->targetVel >= 0);
            assert(tv <= Road::SPEED_LIMIT_MPS);  // the velocity passed here should be coming from controller.getTargetVel().  It should never exceed speed limit.

            assert(int(abs(tv - this->targetVel) *100.0)/100.0 <= int(getAbsMaxAccelT() * DT * 100.0)/100.0 );
            this->targetVel = tv;
        }

        double getTargetVel() const {
            return this->targetVel;
        }

        double getAbsMaxAccelT() const {
            if (accelN < 0)
                return Ego::MAX_ACCEL;
            if (accelN > Ego::MAX_ACCEL)
                return 0.;

            return sqrt(Ego::MAX_ACCEL*Ego::MAX_ACCEL - this->accelN*this->accelN);
        }

    private:
        double accelN = 0;
        double targetVel = 0;

        int count_set_vel = 0;
    };

    TargetV_Controller controller;  // set controller up every tick
};

#endif //PATH_PLANNING_EGO_H

//
// Created by Dimitriy Litvak on 2019-06-12.
//

#include "Car.h"
#include "Ego.h"

using std::cout;
using std::cerr;
using std::endl;

Ego::Ego(Road *rd) : Car() {
    this->road = rd;
    this->lane = Road::MIDDLE_LANE;
}

void Ego::setTelemetry(
    nlohmann::basic_json<map, vector, string, bool, int64_t, uint64_t, double, std::allocator, nlohmann::adl_serializer> &j) {

    if (!this->timerStarted) {
        startTime = std::chrono::system_clock::now();
        this->timerStarted = true;
    }

/* TODO rem
    const std::chrono::duration<double, std::milli> &time_span = std::chrono::high_resolution_clock::now() - this->prevT;
    this->dt = this->dt < 0 ? DT : time_span.count() / 1000.;
*/

    // j[1] is the data JSON object
    // Note: this->targetVel is the desired current speed (target speed).  Telemetry vel. is due to the previous points that the car tries to follow.

    this->x = j[1]["x"];
    this->y = j[1]["y"];
    this->vel = convert_mph_to_ms(j[1]["speed"]);
    this->s = j[1]["s"];
    this->d = j[1]["d"];
    this->yaw = j[1]["yaw"];

    this->vx = this->vel * cos(this->yaw);
    this->vy = this->vel * sin(this->yaw);

    auto curr_lane = (Road::LANE)(d/Road::LANE_WIDTH);
    if (curr_lane != this->lane && this->isInLaneChangeState()) {  //changed lane
        this->lane = curr_lane;
        this->laneCrossedInLCState = true;
    }

    // Previous path data given to the Planner
    auto ppx = j[1]["previous_path_x"];
    this->previous_path_x.clear();
    for (auto & x : ppx)
        this->previous_path_x.push_back(x);

    auto ppy = j[1]["previous_path_y"];
    this->previous_path_y.clear();
    for (auto & y : ppy)
        this->previous_path_y.push_back(y);

    // Previous path's end s and d values
    this->end_path_s = j[1]["end_path_s"];
    this->end_path_d = j[1]["end_path_d"];

    this->controller = {this->previous_path_x, this->previous_path_y, this->controller.getTargetVel()};

    this->other_cars_id_map.clear();
    this->other_cars_lane_map.clear();

    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    auto sensor_fusion = j[1]["sensor_fusion"];
    for (const auto& sf : sensor_fusion) {
        int id = sf[0];
        double x = sf[1];
        double y = sf[2];
        double vx = sf[3];
        double vy = sf[4];

        /* Note: simulator sensor_fusion does not return the correct s&d when approaching s_max because of a
         * simulator bug: https://github.com/udacity/self-driving-car-sim/issues/74
         * x, y coords appear correct.  The code applies getFrenet() to calculate s&d correctly.
        double fs = sf[5];
        double fd = sf[6];
        */

        vector<double> fren = getFrenet(x, y, atan2(vy,vx), this->road->getMap().map_waypoints_x, this->road->getMap().map_waypoints_y);
        double fs = fren[0];
        double fd = fren[1];

        double dist_bw_cars = abs(frenetCircSDistanceBetweenCars(this->s, fs));
        if (dist_bw_cars < Ego::OTHER_CAR_DETECTION_HORIZON && (fd >= 0 && fd <= Road::NUM_LANES * Road::LANE_WIDTH)) {
            Car car = Car(id, x, y, vx, vy, fs, fd, this->road);

            //TODO rem cout << "Lane: " << car.getLane() << ", id: " << car.getId() << ", s: " << car.getS() << ", ego.s: " << this->s << endl;

            this->other_cars_id_map.insert(std::pair<int, Car>(id, &car));

            auto it = this->other_cars_lane_map.find(car.getLane());
            if (it == this->other_cars_lane_map.end()) {
                this->other_cars_lane_map.insert(std::pair<Road::LANE, vector<Car>>(car.getLane(), vector<Car>()));
                it = this->other_cars_lane_map.find(car.getLane());
            }
            it->second.push_back(car);
        }
    }

    // sort this->other_cars_lane_map according to the car frenet S coord in each lane
    for (short l = 0; l < Road::NUM_LANES; ++l) {
        auto it = this->other_cars_lane_map.find((Road::LANE)l);
        if (it != this->other_cars_lane_map.end()) { //if there are cars in the lane sort them by circ frenet distance
            std::sort(it->second.begin(), it->second.end());

            // TODO rem
            cout << "-------- Lane " << it->first << " --------" << endl;
            for (auto &vec_it : it->second) {
                cout << vec_it.getId() << ", s: " << vec_it.getS() << endl;
            }
        }
    }

    this->set_cars_ahead_and_behind();
    this->calculate_lane_speeds();
}

vector<Car::Position> Ego::get_best_path() {
    // Log lap number
    if (this->s > 0.0 && this->s < 10.0 && !this->roadLapUpdated) {
        this->roadLap++;
        this->roadLapUpdated = true;
        cout << "Road Lap " << this->roadLap << endl;
    }

    if (this->s > 10.0)
        this->roadLapUpdated = false;

    //log debug info
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - this->startTime;
    int el_sec = elapsed_seconds.count();
    int min = (int)(el_sec/60) % 60;
    int hrs = (int)(el_sec/3600);
    int sec = el_sec % 60;
    cout << "--------------- Miles: " << convert_meters_to_miles(this->s + (this->roadLap-1)*Road::MAX_S) <<
    ", Time: " << (hrs > 0 ? std::to_string(hrs) + ":" : "") << min << ":" << sec << " --------------- (e.st: " <<
    (int)this->fsm_state << ", e.ln: " << this->lane << ", e.s: " << this->s << ", e.d: " << this->d << ", e.tV: " << this->controller.getTargetVel() << ", e.sfV: " << this->vel << ")" << endl;

    this->recordIncident();

    vector<FSM_STATES > possible_successor_states = get_possible_successor_states();
    double min_traj_cost = INT_MAX;
    Ego::Trajectory bestTraj;
    vector<Car::Position> bestPath;

    cout << "********************************" << endl;

    for (FSM_STATES st : possible_successor_states) {
        const vector<Ego::Trajectory> & possible_traj_for_state = generate_trajectory(st);
        for (auto traj : possible_traj_for_state) {
            double trajCost = calculate_traj_path_cost(traj);

            cout << "state: " << (int)st << ", cost: " << trajCost << endl;

            if (trajCost < min_traj_cost) {
                min_traj_cost = trajCost;
                bestPath = traj.path;
                bestTraj = traj;
            }
        }
    }
    cout << "********************************" << endl;

    // make sure that bestTraj has been set
    assert(bestTraj.target_spline1_s >= 0);

    bool is_potential_colln = false;
    if (bestTraj.potential_collision_car_id >= 0 && bestTraj.target_state == Ego::FSM_STATES::KL) {
        // A rare case of an imminent collision in the best cost trajectory.  FSM weights push Ego to go into KL state.
        Car &colln_car = this->other_cars_id_map[bestTraj.potential_collision_car_id];
        if (abs(colln_car.getD() - this->d) < 0.85 * Road::LANE_WIDTH) {   // 0.85 * 4m leaves about 1m between the cars sideways (car width 2.5m)
            // Collision is imminent in ALL possible successor FSM states.  PLC & LC should have higher collision cost
            // weights than KL. Thus, KL will be a favored state.
            // Simplistic emergency strategy: stay in KL and increase or decrease target speed depending on the adjacent car locations.

            is_potential_colln = true;
            bool dealt_with_colln = true;
            double s_colln_dist = frenetCircSDistanceBetweenCars(colln_car.getS(), this->s);
            if (s_colln_dist < 0) {   // Ego is ahead of colln causing car.  Try to speed up.
                Car &car_ahd = this->cars_ahead[this->lane];
                if (car_ahd.getId() < 0 || frenetCircSDistanceBetweenCars(car_ahd.getS(), this->s) > Ego::MIN_VEHICLE_FOLLOWING_DISTANCE) {
                    // Ego is ahead of the coll-n vehicle AND there's no car ahead or the car ahead is far enough: try to accelerate.
                    bool accelSucc = this->controller.accelerate_max(TargetV_Controller::ACCEL_DIR::PLUS);

                    //TODO rem
                    if (accelSucc) cout << "Dealing with the imminent colln with car id " << colln_car.getId() << ". New targ vel after accel: " << this->controller.getTargetVel() << endl;

                    if (!accelSucc) {
                        Ego::TargetV_Controller::emergency("Car id " + std::to_string(colln_car.getId()) +
                                                           " is causing an imminent collision. Ego can't accelerate; it's already at max acceleration.");
                        // In the real world, I would try to change lane.  Here, I stay in KL state and with the same path & vel and print a warning message.
                        dealt_with_colln = false;
                    }
                } else {
                    Ego::TargetV_Controller::emergency("Car id " + std::to_string(colln_car.getId()) +
                                                       " is causing an imminent collision. Ego can't accelerate; there's a car (id: " +
                                                       std::to_string(car_ahd.getId()) + ") ahead.");
                    // In the real world, I would try to change lane.  Here, I stay in KL state and with the same path & vel and print a warning message.
                    dealt_with_colln = false;
                }
            }
            else {
                // coll-n car is ahead of Ego. Slow down till a stop, if have to.
                bool accelSucc = this->controller.accelerate_max(TargetV_Controller::ACCEL_DIR::MINUS);

                //TODO rem
                if (accelSucc) cout << "Dealing with the imminent colln with car id " << colln_car.getId() << ". New targ vel after decel: " << this->controller.getTargetVel() << endl;

                if (!accelSucc) {
                    Ego::TargetV_Controller::emergency("Car id " + std::to_string(colln_car.getId()) +
                                                       " is causing an imminent collision. Ego can't decelerate; it's already stopped.");
                    // In the real world, I would try to change lane.  Here, I stay in KL state and with the same path & vel and print a warning message.
                    dealt_with_colln = false;
                }
            }

            this->fsm_state = Ego::FSM_STATES::KL;
            if (dealt_with_colln) {
                // non-emergency: KL and accel or decel.
                bestPath = getPath(this->previous_path_x, this->previous_path_y, bestTraj.target_spline1_s, bestTraj.target_d, this->controller.getTargetVel(), Ego::PATH_HORIZON_SEC);
            }
            else {
                // emergency: keep the best vel & path (simple strategy)
                this->controller.setTargetVel(bestTraj.target_v);
            }
        }
    }

    if (!is_potential_colln) { // no imminent coll-n
        bool is_previous_state_LC = this->isInLaneChangeState();
        this->fsm_state = bestTraj.target_state;
        this->controller.setTargetVel(bestTraj.target_v);

        if (!this->isInLaneChangeState() ) {
            // reset the flag here: to indicate the lane change was finished.
            // Used later in gen_lane_change_trajectories to figure out the initial target lane at the start of LC
            this->laneCrossedInLCState = false;

            if (is_previous_state_LC && this->fsm_state == Ego::FSM_STATES::KL) {
                // start KL timer.  The timer is used to deal with excessive lane weaving:
                // stay in KL state for some seconds unless a collision is imminent.
                this->KL_state_start_timepoint = std::chrono::high_resolution_clock::now();
            }
        }
    }

    cout << "BEST STATE: " << (int)this->fsm_state << endl;
    assert(!bestPath.empty());

    const vector<Position> &retPath = (bestPath.size() <= Ego::PATH_HORIZON_TICKS ? bestPath :
        vector<Car::Position>(bestPath.begin(), bestPath.begin() + Ego::PATH_HORIZON_TICKS));
    return retPath;
}

/*
 * Changing lanes in LCL and LCR states is not instantaneous.  The car must finish switching lanes before it can go
 *  to KL state.
 */
vector<Ego::FSM_STATES> Ego::get_possible_successor_states() {
    vector<FSM_STATES> possible_states = {FSM_STATES::KL};

    switch (this->fsm_state) {
        case FSM_STATES::KL:
            if (this->lane == Road::LEFT_LANE || this->lane == Road::MIDDLE_LANE)
                possible_states.push_back(FSM_STATES::PLCR);
            if (this->lane == Road::RIGHT_LANE || this->lane == Road::MIDDLE_LANE)
                possible_states.push_back(FSM_STATES::PLCL);
            break;

        case FSM_STATES::PLCL:
            possible_states.push_back(FSM_STATES::PLCL);
            possible_states.push_back(FSM_STATES::LCL);
            break;

        case FSM_STATES::PLCR:
            possible_states.push_back(FSM_STATES::PLCR);
            possible_states.push_back(FSM_STATES::LCR);
            break;

        case FSM_STATES::LCL:
            possible_states.push_back(FSM_STATES::LCL);
            break;

        case FSM_STATES::LCR:
            possible_states.push_back(FSM_STATES::LCR);
            break;

        case FSM_STATES::CS:
            break;

        default:
            std::cerr << "ERROR: road FSM state is not valid. Exiting.." << endl;
            exit(0);
    }

    return possible_states;
}

// Given a possible next state, generate the appropriate trajectory to realize
//   the next state.
vector<Ego::Trajectory> Ego::generate_trajectory(FSM_STATES next_state) {
    vector<Ego::Trajectory> trajectories;
    if (next_state == FSM_STATES::KL) {
        trajectories = gen_keep_lane_trajectory();
    }
    else if (Ego::isInLaneChangeState(next_state)) {
        trajectories = gen_lane_change_trajectories(next_state);
    }
    else if (Ego::isInPrepLaneChangeState(next_state)) {
        trajectories = gen_prep_lane_change_trajectory(next_state);
    }

    return trajectories;
}

//TODO
vector<Ego::Trajectory> Ego::gen_keep_lane_trajectory() {
    double new_v = get_kinematics(this->lane);
    double sp1 = Car::SPLINE_POINT_1;
    double min_acc_tot = MAXFLOAT;
    Trajectory best_traj;
    for (int i = 0; sp1 + i < Car::SPLINE_POINT_2; i += 4) {
        Trajectory traj = Trajectory(FSM_STATES::KL, this->lane, sp1 + i,
                                     Road::LANE_WIDTH * this->lane + Road::LANE_WIDTH / 2., new_v);
        traj.path = getPath(this->previous_path_x, this->previous_path_y, traj.target_spline1_s, traj.target_d, traj.target_v, (double)Car::PREDICTION_FAR_HORIZON_SEC);
        double accel = get_max_tot_accel(traj);
        if (accel < min_acc_tot) {
            min_acc_tot = accel;
            best_traj = traj;
        }
    }

    return {best_traj};
}

//TODO
vector<Ego::Trajectory> Ego::gen_lane_change_trajectories(Ego::FSM_STATES state) {
    Road::LANE target_change_lane = this->lane;
    if (!this->laneCrossedInLCState) {
        if (state == Ego::FSM_STATES::LCL) {
            target_change_lane = (Road::LANE) (this->lane - 1);
        } else if (state == Ego::FSM_STATES::LCR) {
            target_change_lane = (Road::LANE) (this->lane + 1);
        } else
            throw std::runtime_error(
                "ERROR: gen_lane_change_trajectories state is wrong: " + std::to_string((int) state) + ". Exiting..");
    }

    vector<Ego::Trajectory> lc_traj;
    double sp1 = Car::SPLINE_POINT_1;
    Car::Position pos_infinity = Car::Position();
    pos_infinity.pt_s = pos_infinity.pt_d = -1;
    double target_d = Road::LANE_WIDTH * target_change_lane + Road::LANE_WIDTH / 2.;

/*
    // Create trajectories arriving into the new lane at various coords s (dist_to_mid_change_lane) and times (time_lc)
    for (int i = 0; sp1 + i < Car::SPLINE_POINT_2; i += 4) {
        double sp1_s_dist = sp1 + i;
        double d_dist = abs(this->d - target_d);
        double change_lane_target_dist = sqrt(sp1_s_dist * sp1_s_dist + d_dist * d_dist);
        double min_time = change_lane_target_dist / Road::SPEED_LIMIT_MPS;
        double t_inc = 0.5;

        for (int k = 0; min_time + k*t_inc < Ego::MAX_TOTAL_LANE_CHANGE_TIME_SEC; ++k) {
            double time_lc = min_time + k*t_inc;
            double ego_change_lane_vel = change_lane_target_dist / time_lc; // linear approximation of the velocity along the arc.

            // TODO rem:  ego_change_lane_vel needs to account the increasing acceleration.
            cout << "LC traj targ_sp1_s: " << sp1_s_dist << ", targ_d: " << target_d << ", time_lc: " << time_lc << ", ego_change_lane_vel: " << ego_change_lane_vel << endl;
            assert(int(ego_change_lane_vel*1000)/1000. <= Road::SPEED_LIMIT_MPS);


            // It's important to filter out bad trajectories in advance, as the cost function for proximity and
            // collision only looks at PREDICTION_NEAR_HORIZON_TICKS ahead, which is less than SPLINE_POINT_1 distance.
            vector<Car::Position> target_lane_gap_boundaries;
            if (target_change_lane != this->lane) {
                // Ego is still in the starting lane.

                // Ego can not decrease its speed to that below the car behind in the starting lane if that car is too close.
                Car &car_beh = this->cars_behind[this->lane];
                if (car_beh.getId() >= 0) {
                    double dist_beh = frenetCircSDistanceBetweenCars(this->s, car_beh.getS());
                    
                    //TODO rem
                    assert(dist_beh > 0);
                    
                    if (dist_beh <= Ego::MIN_VEHICLE_FOLLOWING_DISTANCE && ego_change_lane_vel < car_beh.getVel()) {

                        //TODO rem
                        cout << "Car id " << car_beh.getId() << " has higher than ego velocity and is too close behind.  Skipping this trajectory.";

                        continue;
                    }
                    
                }
                
                // Figure out the gaps in the target lane.
                target_lane_gap_boundaries.push_back(pos_infinity);  // gap before the 1st car in the target lane
                auto itr = this->other_cars_lane_map.find(target_change_lane);
                for (Car &c : itr->second) {
                    target_lane_gap_boundaries.push_back(c.position_at(time_lc));
                }
                target_lane_gap_boundaries.push_back(pos_infinity);  // gap after the last car in the target lane
            }
            else {
                // Ego is already in the target lane.  There's only 1 gap between the car behind and the car ahead.
                // Let's see if Ego will fit into the gap in the future.
                Car &car_beh = this->cars_behind[this->lane];
                Car &car_ahd = this->cars_ahead[this->lane];
                Car::Position gap_pos_beh = car_beh.getId() < 0 ? pos_infinity : car_beh.position_at(time_lc);
                Car::Position gap_pos_ahd = car_ahd.getId() < 0 ? pos_infinity : car_ahd.position_at(time_lc);

                target_lane_gap_boundaries.push_back(gap_pos_beh);
                target_lane_gap_boundaries.push_back(gap_pos_ahd);
            }

            // Throw out the trajectories that land Ego in a gap too close to another vehicle.  
            // Unlike collision cost, this code looks at the further horizon.
            // Note: I set ego prediction at max Car::PREDICTION_FAR_HORIZON_SEC all the time, because time_lc is approximate
            // due to the linear velocity approximation: ego_change_lane_vel
            vector<Car::Position> ego_pred = getPath(this->previous_path_x, this->previous_path_y, sp1_s_dist,
                                                     target_d, ego_change_lane_vel, Car::PREDICTION_FAR_HORIZON_SEC);
            for (u_long p = 0; p < target_lane_gap_boundaries.size(); ++p) {
                //todo GAP: aim at the 1 gap before and 1 ahead
            }

            lc_traj.emplace_back(state, target_change_lane, sp1_s_dist,
                                 target_d, ego_change_lane_vel);
        }
    }

    if (lc_traj.empty()) {
        // Ensure at least on trajectory for lane change.  Try to match the target lane speed.
        Car &car_ahd_targ_lane = this->cars_ahead[target_change_lane];
        Car &car_beh_targ_lane = this->cars_behind[target_change_lane];
        double change_lane_speed = this->controller.getTargetVel();

        if (car_ahd_targ_lane.getId() >= 0) {
            // if there's a car ahead, we should try to match its speed in the lane we are changing to
            change_lane_speed = get_kinematics(target_change_lane);
        }
        else if (car_beh_targ_lane.getId() >= 0) {
            // there's no car ahead, only a car behind
            // Note: if there're NO cars ahead or behind OR the car behind has lower velocity than ego,
            // we can switch lane with the current target vel.
            if (car_beh_targ_lane.getVel() > this->controller.getTargetVel())
                change_lane_speed = this->controller.getMaxNewVelocity(TargetV_Controller::ACCEL_DIR::PLUS);
        }

        //TODO rem
        cout << ">> LC traj generation came short generating trajectories. Returning the default trajectory." << endl;

        lc_traj.emplace_back(state, target_change_lane, sp1, target_d, change_lane_speed);

    }
    else {
        // TODO rem
        cout << ">> Exploring " << lc_traj.size() << " LC trajectories." << endl;
        for (auto &traj : lc_traj) {
            cout << "> Traj. sp1: " << traj.target_spline1_s << ", targ_vel: " << traj.target_v << ", targ_lane: " << traj.target_lane << ", ego.lane: " << this->lane << endl;
        }
    }
*/

    Car &car_ahd_targ_lane = this->cars_ahead[target_change_lane];
    Car &car_beh_targ_lane = this->cars_behind[target_change_lane];
    double change_lane_speed = this->controller.getTargetVel();

    if (car_ahd_targ_lane.getId() >= 0) {
        // if there's a car ahead, we should try to match its speed in the lane we are changing to
        change_lane_speed = get_kinematics(target_change_lane);
    }
    else if (car_beh_targ_lane.getId() >= 0) {
        // there's no car ahead, only a car behind
        // Note: if there're NO cars ahead or behind OR the car behind has lower velocity than ego,
        // we can switch lane with the current target vel.
        if (car_beh_targ_lane.getVel() > this->controller.getTargetVel())
            change_lane_speed = this->controller.getMaxNewVelocity(TargetV_Controller::ACCEL_DIR::PLUS);
    }

    Trajectory bestTraj;
    double min_acc = MAXFLOAT;
    for (int i = 0; sp1 + i < Car::SPLINE_POINT_2; i += 4) {
        Trajectory traj = Trajectory(state, target_change_lane, sp1+i, target_d, change_lane_speed);
        traj.path = getPath(this->previous_path_x, this->previous_path_y, traj.target_spline1_s, traj.target_d, traj.target_v, (double)Car::PREDICTION_FAR_HORIZON_SEC);
        if (Ego::get_out_of_lane_time(traj) < Ego::MAX_TIME_OUT_OF_LANE) {
            double accel = get_max_tot_accel(traj);
            if (accel < Ego::MAX_ACCEL && accel < min_acc) {
                min_acc = accel;
                bestTraj = traj;
            }
        }
    }

    if (!bestTraj.path.empty())
        lc_traj.push_back(bestTraj);

    return  lc_traj;
}

// Generate a trajectory to prepare for a lane change.  Mainly, it is to try to match the velocity of the change lane.
//  It also serves as a buffer between KL and LC states.
vector<Ego::Trajectory> Ego::gen_prep_lane_change_trajectory(Ego::FSM_STATES state) {
    Road::LANE prep_for_lane;
    if (state == Ego::FSM_STATES::PLCL) {
        prep_for_lane = (Road::LANE) (this->lane - 1);
    }
    else if (state == Ego::FSM_STATES::PLCR) {
        prep_for_lane = (Road::LANE) (this->lane + 1);
    }
    else
        throw std::runtime_error("ERROR: gen_prep_lane_change_trajectory state is wrong: " + std::to_string((int)state) + ". Exiting..");

    vector<Ego::Trajectory> prepTraj;

    double prep_lane_speed = this->controller.getTargetVel();
    Car &car_ahd = this->cars_ahead[prep_for_lane];
    Car &car_beh = this->cars_behind[prep_for_lane];

    if (car_ahd.getId() >= 0) {
        // if there's a car ahead, we should try to match its speed in the lane we are preparing to change to
        prep_lane_speed = get_kinematics(prep_for_lane);
    }
    else if (car_beh.getId() >= 0) {
        // there's no car ahead, only a car behind
        // Note: if there's NO cars ahead or behind, we can switch lane with the current target vel.
        if (car_beh.getVel() > this->controller.getTargetVel())
            prep_lane_speed = this->controller.getMaxNewVelocity(TargetV_Controller::ACCEL_DIR::PLUS);
    }

    // TODO
    /*double cur_lane_speed = this->lane_speeds[this->lane];
    cout << ">>>>>>>>> Prep change to lane: " << prep_for_lane << " (prep ln speed: " << prep_lane_speed << ", cur ln sp: " << cur_lane_speed << ") <<<<<<<<<" << endl;
    cout << ">>>>>>>>> Ego new speed: " << prep_ego_speed << ", v_ah: " << this->cars_ahead[prep_for_lane].getVel() << " (id: " << this->cars_ahead[prep_for_lane].getId()
    << "), v_beh: " << cars_behind[prep_for_lane].getVel() << " (id: " << cars_behind[prep_for_lane].getId() << ")" << endl;
*/

    Trajectory traj = Trajectory(state, prep_for_lane, (double) Car::SPLINE_POINT_1, Road::LANE_WIDTH * this->lane + Road::LANE_WIDTH / 2., prep_lane_speed);
    traj.path = getPath(this->previous_path_x, this->previous_path_y, traj.target_spline1_s, traj.target_d, traj.target_v, (double)Car::PREDICTION_FAR_HORIZON_SEC);
    prepTraj.push_back(traj);
    return prepTraj;
}


// Gets next timestep kinematics (position, velocity, acceleration)
//   for a given lane. Tries to choose the maximum velocity and acceleration,
//   given other vehicle positions and accel/velocity constraints.
double Ego::get_kinematics(Road::LANE ln) {
    assert(ln == Road::LANE::RIGHT_LANE || ln == Road::LANE::MIDDLE_LANE || ln == Road::LANE::LEFT_LANE );

    double new_velocity;
    Car& vehicle_ahead = this->cars_ahead[ln];

    if (vehicle_ahead.getId() >= 0) {  // if there's a car ahead
        double car_dist = frenetCircSDistanceBetweenCars(vehicle_ahead.getS(), this->s);
        assert(car_dist >= 0);

        double buffer_dist = car_dist - Ego::PREFERRED_VEHICLE_FOLLOWING_DISTANCE;

        cout << "vel_ahead: " << vehicle_ahead.getVel() << ", ego.tag_vel: " << this->controller.getTargetVel() << ", ego.spd: " << this->vel << ", dist_ah: " << car_dist << ", buff_dist: " << buffer_dist << endl;
        cout << "ln: " << ln << ", ego.ln: " << this->lane << endl;

        double vel_diff = this->controller.getTargetVel() - vehicle_ahead.getVel();
        if (buffer_dist > 0) {
            if (vel_diff > 0) {
                double estim_decel = vel_diff*vel_diff / buffer_dist;

                new_velocity = this->controller.getNewVelocity(-estim_decel);
                cout << "EC_buf_gt_0: " << new_velocity << ", ego targ vel: " << this->controller.getTargetVel() << ", est_decel: " << -estim_decel << ", decel: " << -std::min(estim_decel, this->controller.getAbsMaxAccelT()) << endl;
            }
            else {
                new_velocity = this->controller.getMaxNewVelocity(TargetV_Controller::ACCEL_DIR::PLUS);;

                cout << "CE_buf_gt_0: " << new_velocity << ", " << this->controller.getTargetVel() << endl;
            }
        }
        else {
            if (vel_diff > 0) {
                new_velocity = this->controller.getMaxNewVelocity(TargetV_Controller::ACCEL_DIR::MINUS);
                cout << "EC_buf_less_0: " << new_velocity << ", " << this->controller.getTargetVel() << endl;
            }
            else {
                new_velocity = abs(vel_diff) > this->controller.getAbsMaxAccelT()*DT ? this->controller.getTargetVel() : this->controller.getMaxNewVelocity(TargetV_Controller::ACCEL_DIR::MINUS);
                cout << "CE_buf_less_0: new_vel: " << new_velocity << ", targ.vel: " << this->controller.getTargetVel() << ", ego.spd: " << this->vel << endl;
            }
        }
    } else {
        new_velocity = this->controller.getMaxNewVelocity(TargetV_Controller::ACCEL_DIR::PLUS);
    }

    assert(new_velocity >= 0);
    assert(this->vel <= 50);

    return new_velocity;
}

// Returns a true if a vehicle is found behind the current vehicle, false
//   otherwise. The passed reference rVehicle is updated if a vehicle is found.
bool Ego::get_vehicle_behind(Road::LANE ln, Car &rVehicle) {
    double max_s = -1;
    bool found_vehicle = false;
    Car temp_vehicle;
    rVehicle = temp_vehicle;
    for (auto & oc : this->other_cars_id_map) {
        temp_vehicle = oc.second;
        if (temp_vehicle.getLane() == ln && frenetCircSDistanceBetweenCars(temp_vehicle.getS(), this->s) < 0
            && frenetCircSDistanceBetweenCars(temp_vehicle.getS(), max_s) > 0) {
            max_s = temp_vehicle.getS();
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

// Returns a true if a vehicle is found ahead of the current vehicle, false
//   otherwise. The passed reference rVehicle is updated if a vehicle is found.
bool Ego::get_vehicle_ahead(Road::LANE ln, Car &rVehicle) {
    double min_s = -1;
    bool found_vehicle = false;
    Car temp_vehicle;
    rVehicle = temp_vehicle;
    for (auto & traj : this->other_cars_id_map) {
        temp_vehicle = traj.second;
        if (temp_vehicle.getLane() == ln && frenetCircSDistanceBetweenCars(temp_vehicle.getS(), this->s) >= 0
            && (min_s < 0 || frenetCircSDistanceBetweenCars(temp_vehicle.getS(), min_s) < 0)) {
            min_s = temp_vehicle.getS();
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

//TODO this might not be needed
void Ego::calculate_lane_speeds() {
    cout << "========================" << endl;

    for (int i = 0; i < Road::NUM_LANES; ++i) {

        //TODO cout << "Lane " << i;

        double v_ahead = -1.;
        double v_behind = -1.;
        if (this->cars_ahead[i].getId() >= 0) {
            v_ahead = this->cars_ahead[i].getVel();
        }

        //TODO cout << ", cid: " << this->cars_ahead[i].getId() << ", v_ah " << v_ahead;

        if (this->cars_behind[i].getId() >= 0) {
            v_behind = this->cars_behind[i].getVel();
        }

        //TODO cout << ", cid: " << this->cars_behind[i].getId() << ", v_beh " << v_behind;

        if (v_ahead < 0 && v_behind < 0)   // lane is clear of the vehicles at the moment: mark it with speed limit + 1
            this->lane_speeds[i] = Road::SPEED_LIMIT_MPS;
        else if (v_ahead < 0)    // there are vehicles behind only
            if (this->lane == i)
                this->lane_speeds[i] = Road::SPEED_LIMIT_MPS;
            else
                this->lane_speeds[i] = (this->vel >= v_behind - Ego::LANE_CHANGE_TRIGGER_VELOCITY_DIFF_MS || v_behind > Road::SPEED_LIMIT_MPS ? Road::SPEED_LIMIT_MPS : v_behind);
        else {
            // there are vehicles ahead only or both ahead and behind
            this->lane_speeds[i] = v_ahead > Road::SPEED_LIMIT_MPS ? Road::SPEED_LIMIT_MPS : v_ahead;
        }

        //TODO  cout << ", lsp: ";
        cout << this->lane_speeds[i] << "||";
    }

    cout << endl << "========================" << endl;
}

void Ego::set_cars_ahead_and_behind() {
    for (int i = 0; i < Road::NUM_LANES; ++i) {
        Car car_ahd, car_beh;

        auto it = this->other_cars_lane_map.find((Road::LANE)i);
        if (it != this->other_cars_lane_map.end()) {
            // cars in the lane
            for (const auto &c : it->second) {
                double ds = frenetCircSDistanceBetweenCars(this->s, c.getS());
                if (ds >= 0) // Ego is ahead of the car c
                    car_beh = c;
                else {
                    car_ahd = c;
                    break;
                }
            }
        }
        this->cars_ahead[i] = car_ahd;
        this->cars_behind[i] = car_beh;

        //TODO rem
        if (car_ahd.getId() >= 0)
            cout << "Ahead: Lane: " << car_ahd.getLane() << ", id: " << car_ahd.getId() << ", c.s: " << car_ahd.getS()
            << ", c.d: " << car_ahd.getD() << ", c.v: " << car_ahd.getVel()
            << ", ego.s: " << this->s << ", ego.d: " << this->d << ", ego.v: " << this->vel << endl;

        //TODO rem
        if (car_beh.getId() >= 0)
            cout << "Behind: Lane: " << car_beh.getLane() << ", id: " << car_beh.getId() << ", s: " << car_beh.getS()
                << ", c.d: " << car_beh.getD() << ", c.v: " << car_beh.getVel()
                << ", ego.s: " << this->s << ", ego.d: " << this->d << ", ego.v: " << this->vel << endl;
    }
}

// Calculate the cost of the trajectory's path and return the path by reference via ret_traj_path
double
Ego::calculate_traj_path_cost(Ego::Trajectory &trajectory) {
    double cost = 0.0;

    auto& costType_weight_map = COST_WEIGHT_MAP.at(trajectory.target_state);
    for (auto& ctw : costType_weight_map) {
        Ego::COST_TYPES ct = ctw.first;
        double weight = ctw.second;
        double new_cost = weight * COST_FUNC_MAP.at(ct)(*this, trajectory);
        cost += new_cost;
    }

    return cost;
}

double Ego::inefficiency_cost(const Ego::Trajectory &trajectory) {
    double maxSpeed = this->lane_speeds[trajectory.target_lane];
    int maxSpeedLane = trajectory.target_lane;
    for (int i=0; i < Road::NUM_LANES; ++i) {
        // in order to register more efficient lane, the velocity difference has to be significant: > LANE_CHANGE_TRIGGER_VELOCITY_DIFF_MS
        if (this->lane_speeds[i] > maxSpeed + Ego::LANE_CHANGE_TRIGGER_VELOCITY_DIFF_MS) {
            maxSpeed = this->lane_speeds[i];
            maxSpeedLane = i;
        }
    }

    assert(maxSpeed > 0);

    double cost = 0;
    bool isEgoInLaneChangeState = this->isInLaneChangeState();
    // case 1: in any state, the max speed has to be significantly higher than the current lane
    // case 2: in LC state, we are committed to changing lanes unless a collision is imminent. After starting LC,
    // inefficiency is calculated ONLY based on the target lane vs the starting lane.
    if (maxSpeed - this->lane_speeds[this->lane] > Ego::LANE_CHANGE_TRIGGER_VELOCITY_DIFF_MS || isEgoInLaneChangeState) {
        cost = (double) abs(maxSpeedLane - (int) trajectory.target_lane) / (Road::NUM_LANES - 1.);
    }
    cout << "inefficiency_cost: lane speed cost: " << cost;

    // this part adjusts behaviour in which Ego tries to match the speed of the target lane in PLC state before it
    // catches up to the vehicle far ahead in its own lane.
    if (trajectory.target_state == Ego::FSM_STATES::KL && !isEgoInLaneChangeState) {
        Car& car_ahd = this->cars_ahead[this->lane];

        if (car_ahd.getId() >= 0) {
            double car_ahd_dist = frenetCircSDistanceBetweenCars(car_ahd.getS(), this->s);
            if (car_ahd_dist > Ego::PREFERRED_VEHICLE_FOLLOWING_DISTANCE) {
                // there's a car ahead further then preferred distance.  Inefficiency drops, as Ego has space to maneuver.
                double following_cost = 1 - 0.7 * (car_ahd_dist - Ego::PREFERRED_VEHICLE_FOLLOWING_DISTANCE)
                        / (Ego::OTHER_CAR_DETECTION_HORIZON - Ego::PREFERRED_VEHICLE_FOLLOWING_DISTANCE);
                cout << ", following_cost: " << following_cost;
                cost *= following_cost;
            }
        }
    }
    cout << ", final cost: " << cost << endl;
    return cost;
}

// 1 - collision. 0 - distance is greater than min
double Ego::collision_and_proximity_cost(Ego::Trajectory &trajectory) {
    double max_cost = 0;
    const vector<Car::Position> &ego_path = trajectory.path;
    for (const auto & oc_it : this->other_cars_id_map) {
        const vector<Car::Position> &oc_pred = oc_it.second.getPrediction();
        
        assert(ego_path.size() >= Ego::PREDICTION_NEAR_HORIZON_TICKS && oc_pred.size() >= Ego::PREDICTION_NEAR_HORIZON_TICKS);

        double cost = 0;
        //check for collision in the range of PREDICTION_NEAR_HORIZON_SEC ahead
        for (int tk = 0; tk < Ego::PREDICTION_NEAR_HORIZON_TICKS; ++tk) {
            double es = ego_path[tk].pt_s;
            double ed = ego_path[tk].pt_d;

            double cs = oc_pred[tk].pt_s;
            double cd = oc_pred[tk].pt_d;
            auto ego_lane = (Road::LANE)(ed / Road::LANE_WIDTH);
            double delta_s = frenetCircSDistanceBetweenCars(es, cs);  // if delta_s > 0, ego is ahead of the car
            double abs_delta_s = abs(delta_s);
            double delta_d = ed - cd;
            double abs_delta_d = abs(delta_d);

            // collision cost
            if (abs_delta_s < Ego::IN_LANE_COLLISION_DISTANCE && abs_delta_d < Ego::SIDE_COLLISION_DISTANCE) {
                trajectory.potential_collision_car_id = oc_it.first;
                cout << "COLLN collision_and_proximity_cost: 1, car id: " << oc_it.first << endl;
                return 1.;
            }
            else if (abs_delta_s < Ego::MIN_VEHICLE_FOLLOWING_DISTANCE) {
                bool isLaneChangeTrajState = Ego::isInLaneChangeState(trajectory.target_state);
                if (ego_lane == oc_it.second.getLane() || (isLaneChangeTrajState && trajectory.target_lane == oc_it.second.getLane()))
                    cost = 1. - abs_delta_s / Ego::MIN_VEHICLE_FOLLOWING_DISTANCE;
            }

            if (cost > max_cost)
                max_cost = cost;
        }

        //cout << "cid: " << pred.first << ", dist s: " << fabs(pred.second[0].getS() - this->s) << ", collision/proximity cost: " << max_cost << endl;
    }

    cout << "collision_and_proximity_cost: " << max_cost << endl;
    assert(max_cost >= 0 && max_cost < 1.01);

    return max_cost;
}

// Cost: 0 - 1.  This cost increase forces the car towards LC state.  The closer ego matches the target lane speed,
// the more likely it is to go into LC state.
double Ego::lane_change_readiness_cost(const Ego::Trajectory &trajectory) {
    if (!this->isInPrepLaneChangeState())
        return 0;   //cost function is only activated when Ego is in PLC state

    Road::LANE intended_lane = trajectory.target_lane;
    Car &car_ahd = this->cars_ahead[intended_lane];
    Car &car_beh = this->cars_behind[intended_lane];

    if (car_beh.getVel() > Road::SPEED_LIMIT_MPS) {
        cout << "lane_change_readiness_cost: 0." << endl;
        // car behind is going too fast.  Don't get ahead of it.
        return 0.;
    }

    if (car_ahd.getId() < 0 &&
        (car_beh.getId() < 0 || car_beh.getVel() < this->vel)) {
        cout << "lane_change_readiness_cost: 1." << endl;
        // no vehicles ahead or behind OR the vehicle behind is far enough.
        return 1.;
    }

    double cost = exp(-abs(this->controller.getTargetVel() - this->lane_speeds[intended_lane]));

    cout << "lane_change_readiness_cost: " << cost << endl;

    return cost;
}

// This cost is calculated only for PLC states in order to prevent KL state going into PLC before the car
// reaches the center of the lane.  Cost 1 - 2m from lane cntr, 0 - at lane ctr.
double Ego::lane_off_center_cost(const Ego::Trajectory &trajectory) {
    assert(trajectory.target_state == Ego::FSM_STATES::PLCR || trajectory.target_state == Ego::FSM_STATES::PLCL);

    double cost = 0;
    double half_lane_width = Road::LANE_WIDTH / 2.;
    double mid_lane = this->lane*Road::LANE_WIDTH + half_lane_width;
    cost  = abs(this->d - mid_lane) / half_lane_width;

    cout << "off_center_lane_cost: " << cost << endl;
    return cost;
}

/**
 * After LC, try to keep Ego in KL state around Ego::MIN_KL_TIME_AFTER_LC_MILLIS.  Cost=1 when
 * Ego::MIN_KL_TIME_AFTER_LC_MILLIS timer starts; cost=0 after Ego::MIN_KL_TIME_AFTER_LC_MILLIS milliseconds have passed.
 *
 * @param trajectory
 * @param traj_path
 * @param other_car_predictions
 * @return cost depending on number millis passed on the timer.
 */
double Ego::lane_weaving_cost(const Ego::Trajectory &trajectory) {
    double cost = 0;

    std::chrono::duration<double, std::milli> KL_state_millis =  std::chrono::high_resolution_clock::now() - this->KL_state_start_timepoint;
    long millis = KL_state_millis.count();

    if (millis < Ego::MIN_KL_TIME_AFTER_LC_MILLIS && this->fsm_state == Ego::FSM_STATES::KL &&
        Ego::isInPrepLaneChangeState(trajectory.target_state))
        cost = (double)(Ego::MIN_KL_TIME_AFTER_LC_MILLIS - millis)/Ego::MIN_KL_TIME_AFTER_LC_MILLIS;

    cout << "lane_weaving_cost: " << cost << endl;
    return cost;
}

//TODO rem? Excessive jerk and acceleration cost. 1 - jerk exceeds 10 m/s^3 or accle > 10/m/s^2
/*double Ego::out_of_lane_and_accel_spline_cost(const Ego::Trajectory &trajectory) {
    double divider_touch_d1, divider_touch_d2;

    if (trajectory.target_state == Ego::FSM_STATES::LCL) {
        Road::LANE start_lane = this->lane;
        if (this->laneCrossedInLCState)
            start_lane = (Road::LANE) (start_lane + 1);
        divider_touch_d1 = start_lane * Road::LANE_WIDTH - Ego::SIDE_COLLISION_DISTANCE/2.;
        divider_touch_d2 = start_lane * Road::LANE_WIDTH + Ego::SIDE_COLLISION_DISTANCE/2.;
    } else if (trajectory.target_state == Ego::FSM_STATES::LCR) {
        divider_touch_d1 = trajectory.target_lane * Road::LANE_WIDTH - Ego::SIDE_COLLISION_DISTANCE/2.;
        divider_touch_d2 = trajectory.target_lane * Road::LANE_WIDTH + Ego::SIDE_COLLISION_DISTANCE/2.;
    }
    else if (trajectory.target_state == Ego::FSM_STATES::KL) {
        divider_touch_d1 = trajectory.target_lane * Road::LANE_WIDTH + Ego::SIDE_COLLISION_DISTANCE/2.;
        divider_touch_d2 = (trajectory.target_lane+1) * Road::LANE_WIDTH - Ego::SIDE_COLLISION_DISTANCE/2.;
    }
    else
        throw std::runtime_error(
            "ERROR: out_of_lane_and_accel_spline_cost state " + std::to_string((int) trajectory.target_state) + " is invalid. Exiting..");

    assert(divider_touch_d1 >=0 && divider_touch_d1 < Road::NUM_LANES*Road::LANE_WIDTH);
    assert(divider_touch_d2 >=0 && divider_touch_d2 < Road::NUM_LANES*Road::LANE_WIDTH);


    double max_atot = 0;
    double lc_tick_cnt = 0;

    //TODO rem
    cout << ">>> accel&time <<<" << endl;

    // calculate max total acceleration of the path and the time the ego is touching the dividing line
    for (u_int p = 0; p < Car::PREDICTION_FAR_HORIZON_TICKS - 2; ++p) {
        Car::Position pos1 = traj_path[p];
        Car::Position pos2 = traj_path[p+1];
        Car::Position pos3 = traj_path[p+2];

        // accel limit
        double R = Ego::estimate_curve_radius(pos1, pos2, pos3);
        double v1 = distance(pos1.pt_x, pos1.pt_y, pos2.pt_x, pos2.pt_y)/DT;
        double v2 = distance(pos2.pt_x, pos2.pt_y, pos3.pt_x, pos3.pt_y)/DT;
        double ave_v = (v1 + v2)/2.;

        double an = ave_v*ave_v/R;
        double at = abs(v2 - v1)/DT;
        double atot = sqrt(an*an + at*at);

        //TODO rem
        //cout << "an: " << an << ", at: " << at << ", atot: " << atot << endl;

        if (atot > max_atot)
            max_atot = atot;

        // time limit
        if ((Ego::isInLaneChangeState(trajectory.target_state) && pos1.pt_d >= divider_touch_d1 && pos1.pt_d <= divider_touch_d2)
            || (trajectory.target_state == Ego::FSM_STATES::KL && (pos1.pt_d <= divider_touch_d1 || pos1.pt_d >= divider_touch_d2)))
            lc_tick_cnt++;

        if (max_atot > Ego::MAX_ACCEL) {
            cout << "St: " << int(trajectory.target_state) << "| out_of_lane_and_accel_spline_cost (Max accel exceeded): " << max_atot << endl;
            return 1.;
        }

        double lc_time_sec = lc_tick_cnt * DT;

        //TODO rem
        //cout << "|| pos1.pt_d: " << pos1.pt_d << ", div_d1: " <<  divider_touch_d1 << ", div_d2: " << divider_touch_d2
        //<< ", lc_tick_cnt: " << lc_tick_cnt << ", lc_time_sec: " << lc_time_sec << endl;


        if (lc_time_sec > Ego::MAX_TOTAL_LANE_CHANGE_TIME_SEC) {
            cout << "out_of_lane_and_accel_spline_cost (lc time exceeded): " << lc_time_sec << endl;
            return 1.; 
        }
    }

    double acost = max_atot / Ego::MAX_ACCEL;
    double tcost = lc_tick_cnt * DT / Ego::MAX_TOTAL_LANE_CHANGE_TIME_SEC;
    double cost = (acost + tcost) / 2.;

    cout << "out_of_lane_and_accel_spline_cost: acost: " << acost << ", tcost: " << tcost << ", cost: " << cost << endl;
    return cost;
}*/

void Ego::recordIncident() {
    // collision
    for (auto const & oc : this->other_cars_id_map) {
        double es = this->s;
        double ed = this->d;

        Car car = oc.second;
        double cs = car.getS();
        double cd = car.getD();
        double delta_s = abs(frenetCircSDistanceBetweenCars(es, cs));
        double delta_d = abs(ed - cd);

        /*TODO cout << "colln?: id: " << car.getId() << ", ds: " << delta_s << ", dd: " << delta_d << ", c.ln: " << car.getLane()
         << ", eg.s: " << this->s << ", eg.d: " << this->d << ", e.lane: " << this->lane << ", elane: " << ego_lane << endl;*/

        if (delta_s < Ego::IN_LANE_COLLISION_DISTANCE && delta_d < Ego::SIDE_COLLISION_DISTANCE) {
            std::cerr << "COLLISION with car id:" << car.getId() << ".  Exiting.." << endl;
            exit(0);
        }
    }
}

// Estimate curve radius based on 3 points.  Use Heron's formula to find the triangle's height.  The height is used
// to estimate the arc radius.
double
Ego::calculateCurveRadius(const double x0, const double y0, const double x1, const double y1, const double x2, const double y2) {
    double d1 = distance(x0, y0, x1, y1);
    double d2 = distance(x1, y1, x2, y2);
    double w = distance(x0, y0, x2, y2);

    // use Heron's formula to obtain height of the triangle between 3 points
    double hp = (d1 + d2 + w)/2.;   // half perimeter
    double area = sqrt(hp * (hp - d1) * (hp - d2) * (hp - w));  // Heron's formula for a triangle area
    double h = 2 * area / w;   // height of triangle
    double arc_r = h/2. + w*w/(8.*h);    // radius of an arc formula based on a chord theorem: https://www.mathopenref.com/arcradius.html

    if (isnan(arc_r))
        arc_r = INFINITY;
    return arc_r;
}

bool Ego::isInLaneChangeState() {
    return Ego::isInLaneChangeState(this->fsm_state);
}

bool Ego::isInLaneChangeState(const Ego::FSM_STATES st) {
    return st == Ego::FSM_STATES::LCL || st == Ego::FSM_STATES::LCR;
}

bool Ego::isInPrepLaneChangeState() {
    return Ego::isInLaneChangeState(this->fsm_state);
}
bool Ego::isInPrepLaneChangeState(const Ego::FSM_STATES st) {
    return st == Ego::FSM_STATES::PLCL || st == Ego::FSM_STATES::PLCR;
}

double Ego::estimate_curve_radius(const Car::Position p1, const Car::Position p2, const Car::Position p3) {
    double phi1 = atan2(p2.pt_y - p1.pt_y, p2.pt_x - p1.pt_x);
    double phi2 = atan2(p3.pt_y - p2.pt_y, p3.pt_x - p2.pt_x);
    double dphi_rad = abs(phi2 - phi1);
    double dx = distance(p1.pt_x, p1.pt_y, p3.pt_x, p3.pt_y);

    double R = dx / dphi_rad;
    return R;
}

double Ego::get_max_tot_accel(const Trajectory &trajectory) {
    double max_atot = 0;
    
    // calculate max total acceleration of the path 
    for (u_int p = 0; p < Car::PREDICTION_FAR_HORIZON_TICKS - 2; ++p) {
        Car::Position pos1 = trajectory.path[p];
        Car::Position pos2 = trajectory.path[p+1];
        Car::Position pos3 = trajectory.path[p+2];

        // accel limit
        double R = Ego::estimate_curve_radius(pos1, pos2, pos3);
        double v1 = distance(pos1.pt_x, pos1.pt_y, pos2.pt_x, pos2.pt_y)/DT;
        double v2 = distance(pos2.pt_x, pos2.pt_y, pos3.pt_x, pos3.pt_y)/DT;
        double ave_v = (v1 + v2)/2.;

        double an = ave_v*ave_v/R;
        double at = abs(v2 - v1)/DT;
        double atot = sqrt(an*an + at*at);

        //TODO rem
//        if (atot > MAX_ACCEL)
//            cout << "an: " << an << ", at: " << at << ", atot: " << atot << endl;

        if (atot > max_atot)
            max_atot = atot;
     }

    return max_atot;

}

double Ego::get_out_of_lane_time(const Ego::Trajectory &trajectory) {
    double divider_touch_d1, divider_touch_d2;

    if (trajectory.target_state == Ego::FSM_STATES::LCL) {
        Road::LANE start_lane = this->lane;
        if (this->laneCrossedInLCState)
            start_lane = (Road::LANE) (start_lane + 1);
        divider_touch_d1 = start_lane * Road::LANE_WIDTH - Ego::SIDE_COLLISION_DISTANCE/2.;
        divider_touch_d2 = start_lane * Road::LANE_WIDTH + Ego::SIDE_COLLISION_DISTANCE/2.;
    } else if (trajectory.target_state == Ego::FSM_STATES::LCR) {
        divider_touch_d1 = trajectory.target_lane * Road::LANE_WIDTH - Ego::SIDE_COLLISION_DISTANCE/2.;
        divider_touch_d2 = trajectory.target_lane * Road::LANE_WIDTH + Ego::SIDE_COLLISION_DISTANCE/2.;
    }
    else if (trajectory.target_state == Ego::FSM_STATES::KL) {
        divider_touch_d1 = trajectory.target_lane * Road::LANE_WIDTH + Ego::SIDE_COLLISION_DISTANCE/2.;
        divider_touch_d2 = (trajectory.target_lane+1) * Road::LANE_WIDTH - Ego::SIDE_COLLISION_DISTANCE/2.;
    }
    else
        throw std::runtime_error(
            "ERROR: out_of_lane_and_accel_spline_cost state " + std::to_string((int) trajectory.target_state) + " is invalid. Exiting..");

    assert(divider_touch_d1 >=0 && divider_touch_d1 < Road::NUM_LANES*Road::LANE_WIDTH);
    assert(divider_touch_d2 >=0 && divider_touch_d2 < Road::NUM_LANES*Road::LANE_WIDTH);

    // calculate the time the ego is touching the dividing line
    double lc_tick_cnt = 0;
    for (u_int p = 0; p < Car::PREDICTION_FAR_HORIZON_TICKS - 2; ++p) {
        Car::Position pos1 = trajectory.path[p];
        // time limit
        if ((Ego::isInLaneChangeState(trajectory.target_state) && pos1.pt_d >= divider_touch_d1 && pos1.pt_d <= divider_touch_d2)
            || (trajectory.target_state == Ego::FSM_STATES::KL && (pos1.pt_d <= divider_touch_d1 || pos1.pt_d >= divider_touch_d2)))
            lc_tick_cnt++;

        //TODO rem
        //cout << "|| pos1.pt_d: " << pos1.pt_d << ", div_d1: " <<  divider_touch_d1 << ", div_d2: " << divider_touch_d2
        //<< ", lc_tick_cnt: " << lc_tick_cnt << ", lc_time_sec: " << lc_time_sec << endl;
    }
    
    return lc_tick_cnt * DT;
}





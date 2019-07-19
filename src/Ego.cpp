//
// Created by Dimitriy Litvak on 2019-06-12.
//

#include "Car.h"
#include "Ego.h"

using std::cout;
using std::endl;

Ego::Ego(Road *rd) : Car() {
    this->road = rd;
    this->lane = Road::MIDDLE_LANE;

    this->targetVel = MAX_ACCEL * DT;
}

void Ego::setTelemetry(
    nlohmann::basic_json<map, vector, string, bool, int64_t, uint64_t, double, std::allocator, nlohmann::adl_serializer> &j) {

    // j[1] is the data JSON object
    // Note: this->targetVel is the desired current speed (target speed).  Telemetry vel. is due to the previous points that the car tries to follow.

    this->x = j[1]["x"];
    this->y = j[1]["y"];
    
    if (this->prev_vel != 0)
        this->prev_accel = getEstimatedAccel();
    
    this->prev_vel = this->vel;
    this->vel = convert_mph_to_ms(j[1]["speed"]);
    this->s = j[1]["s"];
    this->d = j[1]["d"];
    this->yaw = j[1]["yaw"];

    auto curr_lane = (Road::LANE)(d/Road::LANE_WIDTH);
    if (curr_lane != this->lane) {  //changed lane
        this->lane = curr_lane;
        this->laneChangeFinished = true;
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

    this->other_cars.clear();

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

        Car car = Car(id, x, y, vx, vy, fs, fd, this->road);

        //cout << "sensor_fusion: id: " << id << ", fus_s: " << fs << ", fus_d: " << fd << ", der_s: " << fren[0] << ", der_d: " << fren[1] << endl;

        double dist_bw_cars = abs(Ego::frenetCircSDistanceBetweenCars(this->s, car.getS()));
        if (dist_bw_cars < Ego::OTHER_CAR_DETECTION_HORIZON) {
            //cout << "Lane: " << car.getLane() << ", id: " << car.getId() << ", s: " << car.getS() << ", ego.s: " << this->s << endl;

            this->other_cars.insert(std::pair<int, Car>(id, &car));
        }
    }
}

double Ego::getEstimatedAccel() const { return (vel - prev_vel) / DT; }

Ego::Path Ego::get_best_path() {
    // Log lap number
    if (this->s > 0.0 && this->s < 1.0 && !this->roadLapUpdated) {
        this->roadLap++;
        this->roadLapUpdated = true;
        cout << "Road Lap " << this->roadLap << endl;
    }

    if (this->s > 1.0)
        this->roadLapUpdated = false;

    //log debug info
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - this->startTime;
    int el_sec = elapsed_seconds.count();
    int min = (int)(el_sec/60) % 60;
    int hrs = (int)(el_sec/3600);
    int sec = el_sec % 60;
    cout << "--------------- Miles: " << convert_meters_to_miles(this->s + (this->roadLap-1)*Road::MAX_S) << ", Time: " << (hrs > 0 ? std::to_string(hrs) + ":" : "") << min << ":" << sec << " ---------------" << endl;

    this->recordIncident();

    this->set_cars_ahead_and_behind();
    this->calculate_lane_speeds();

    map<int, vector<Car>> predictions;
    for (auto & other_car : this->other_cars) {
        predictions[other_car.first] = generate_prediction(other_car.second);
    }

    vector<FSM_STATES > possible_successor_states = get_possible_successor_states();
    double min_traj_cost = INT_MAX;
    Ego::Trajectory bestTraj;
    Ego::Path bestPath;
    cout << "********************************" << endl;
    for (FSM_STATES st : possible_successor_states) {
        const vector<Ego::Trajectory> & possible_traj_for_state = generate_trajectory(st);
        for (auto & traj : possible_traj_for_state) {
            Ego::Path retPath;
            double trajCost = calculate_traj_path_cost(traj, predictions, retPath);

            cout << "state: " << (int)st << ", cost: " << trajCost << endl;

            if (trajCost < min_traj_cost) {
                min_traj_cost = trajCost;
                bestPath = retPath;
                bestTraj = traj;
            }
        }
    }
    cout << "@@@@@@@@@@@@@@@@@@@@@" << endl;

    if (bestTraj.s >= 0) {    // s will be > 0 99.9% of times on non-dummy trajectories
        this->fsm_state = bestTraj.state;
        this->targetVel = bestTraj.v;

        if (this->fsm_state != Ego::FSM_STATES::LCL && this->fsm_state != Ego::FSM_STATES::LCR) {
            this->laneChangeFinished = false;
        }
    }
    cout << "STATE: " << (int)this->fsm_state << endl;

    assert(!bestPath.pts_x.empty() && bestPath.pts_x.size() == bestPath.pts_y.size());

    //TODO this  might be wrong to limit path like this
    vector<double> ptsx(bestPath.pts_x.begin(), bestPath.pts_x.begin() + Ego::PATH_HORIZON);
    vector<double> ptsy(bestPath.pts_y.begin(), bestPath.pts_y.begin() + Ego::PATH_HORIZON);

    return Ego::Path {ptsx, ptsy};
}

// get the path for a possible kinematics/trajectory
Ego::Path Ego::getPath(const Ego::Trajectory & traj) {
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
        prev_ref_x = this->previous_path_x[prev_size - 2];
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
    vector<double> next_wp0 = getXY(ref_s + SPLINE_POINT_1, traj.d, map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);
    vector<double> next_wp1 = getXY(ref_s + SPLINE_POINT_2, traj.d, map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);
    vector<double> next_wp2 = getXY(ref_s + SPLINE_POINT_3, traj.d, map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // transform into car's coord system
    for (int p = 0; p < ptsx.size(); ++p) {
        double shift_x = ptsx[p] - ref_x;
        double shift_y = ptsy[p] - ref_y;

        //cout << "x,y: " << ptsx[p] << "," << ptsy[p] << "; shift: " << shift_x << "," << shift_y << "; yaw, cos, sin: " << ref_yaw << "," << cos(ref_yaw)<< "," << sin(ref_yaw) << ", s: " << car_s << endl;

        ptsx[p] = shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw);
        ptsy[p] = shift_y * cos(ref_yaw) - shift_x * sin(ref_yaw);
    }

    // add points to the vector returned to the sim
    Path path;

    // add previous path points to the next set, so that the total path is continuous
    for (int p = 0; p < prev_size; ++p) {
        double px = previous_path_x[p];
        double py = previous_path_y[p];
        path.pts_x.push_back(px);
        path.pts_y.push_back(py);

        vector<double> fren = getFrenet(px, py, ref_yaw, map.map_waypoints_x, map.map_waypoints_y);
        path.pts_s.push_back(fren[0]);
        path.pts_d.push_back(fren[1]);
    }

    tk::spline spl;
    spl.set_points(ptsx, ptsy);

    double target_x = SPLINE_POINT_1;
    double target_y = spl(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double vel = (traj.v >= 0 ? traj.v : this->targetVel);   
    if (vel<= 0)
        vel = 0.001;  //prevent division by zero and illegal going bkwd
    double inc = target_x / (target_dist / (vel * DT));

    //cout << "traj_vel: " << (traj.v >= 0 ? std::to_string(traj.v) : "none") << ", tar_vel_after: " << this->targetVel << endl;

    for (int p = 1; p <= PREDICTION_HORIZON - prev_size; ++p) {
        double x_pnt = inc * p;
        double y_pnt = spl(x_pnt);

        // transform from car's coord system into global map
        double rot_x = x_pnt * cos(ref_yaw) - y_pnt * sin(ref_yaw);
        double rot_y = y_pnt * cos(ref_yaw) + x_pnt * sin(ref_yaw);

        double px = rot_x + ref_x;
        double py = rot_y + ref_y;
        path.pts_x.push_back(px);
        path.pts_y.push_back(py);

        vector<double> fren = getFrenet(px, py, ref_yaw, map.map_waypoints_x, map.map_waypoints_y);
        path.pts_s.push_back(fren[0]);
        path.pts_d.push_back(fren[1]);
    }

    /*cout << ">>>>>>>>>>>> Prediction EGO <<<<<<<<<<<<<<" << endl;
    for (int p = 0; p < PREDICTION_HORIZON; ++p) {
        cout << "id: EGO, pred.x: " << path.pts_x[p] << ", pred.y: " << path.pts_y[p] << ", pred.s: "
            << path.pts_s[p] << ", pred.d: " << path.pts_d[p] << endl;
    }
    cout << ">>>>>>>>>>>> Prediction_end <<<<<<<<<<<<<<" << endl;
*/

    return path;
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
    else if (next_state == FSM_STATES::LCL || next_state == FSM_STATES::LCR) {
        trajectories = gen_lane_change_trajectories(next_state);
    }
    else if (next_state == FSM_STATES::PLCL || next_state == FSM_STATES::PLCR) {
        trajectories = gen_prep_lane_change_trajectory(next_state);
    }

    return trajectories;
}

// Generates predictions for non-ego vehicles to be used in trajectory
//   generation for the ego vehicle.
vector<Car> Ego::generate_prediction(Car &c) {
    vector<Car> prediction;
    double t = DT;
    //cout << ">>>>>>>>>>>> Prediction <<<<<<<<<<<<<<" << endl;
    for (int p=0; p < PREDICTION_HORIZON; ++p) {
        prediction.push_back(c.position_at(t));
        t += DT;
    }
    //cout << ">>>>>>>>>>>> Prediction_end <<<<<<<<<<<<<<" << endl;
    return prediction;
}

//TODO
vector<Ego::Trajectory> Ego::gen_keep_lane_trajectory() {
    vector<Ego::Trajectory> trajectory;

    vector<double> kinematics = get_kinematics(this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    Ego::Trajectory end_pnt(FSM_STATES::KL, this->lane, Ego::PATH_HORIZON, Road::LANE_WIDTH*this->lane + Road::LANE_WIDTH/2., new_v);
    trajectory.push_back(end_pnt);

    return trajectory;
}

//TODO
vector<Ego::Trajectory> Ego::gen_lane_change_trajectories(Ego::FSM_STATES state) {
    Road::LANE change_lane = this->lane;
    if (!this->laneChangeFinished) {
        if (state == Ego::FSM_STATES::LCL) {
            change_lane = (Road::LANE) (this->lane - 1);
        } else if (state == Ego::FSM_STATES::LCR) {
            change_lane = (Road::LANE) (this->lane + 1);
        } else
            throw std::runtime_error(
                "ERROR: gen_lane_change_trajectories state is wrong: " + std::to_string((int) state) + ". Exiting..");
    }

/*    for (auto& c : this->cars_ahead) {
        cout << "Ahd: cid: " << c.getId() << ", c.s:" << c.getS() << ", ego.s: " << this->s << endl;
    }
    for (auto& c : this->cars_behind) {
        cout << "Behd: cid: " << c.getId() << ", c.s:" << c.getS() << ", ego.s: " << this->s << endl;
    }*/

    //TODO account for veh behind
    vector<double> kinematics = get_kinematics(change_lane);
    double new_s = kinematics[0];
    double new_ego_speed = kinematics[1];
    double new_a = kinematics[2];

    vector<Ego::Trajectory> lct;
    for (int i = 0; i < SPLINE_POINT_2; i+= 8) {
        lct.emplace_back(state, change_lane, Ego::PATH_HORIZON + i, Road::LANE_WIDTH * change_lane + Road::LANE_WIDTH / 2., new_ego_speed);
    }
    return lct;
}


//TODO
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

    // don't change the lane if the velocity is not significantly higher than that in the current lane.
//    double cur_lane_speed = this->lane_speeds[this->lane];
//    if (prep_lane_speed < cur_lane_speed + Ego::LANE_CHANGE_TRIGGER_VELOCITY_MS)
//        return prepTraj;

    //TODO collision cost calc could take care of that
    // Abort prep for lane change if getting too close to the vehicle?
/*
    Car &car_ah = this->cars_ahead[this->lane];
    if (car_ah.getId() >= 0) {
        double dist_ah = abs(Ego::frenetSDistanceBetweenCars(car_ah.getS(), this->s));
        if (dist_ah < Ego::MIN_VEHICLE_FOLLOWING_DISTANCE) {
            cout << "Ah: Abort prep state: " << (int) state << " with dist_ah: " << dist_ah << endl;
            return prepTraj;
        }
    }

    Car &car_beh = this->cars_behind[this->lane];
    if (car_beh.getId() >= 0) {
        double dist_beh = abs(Ego::frenetCircSDistanceBetweenCars(car_ah.getS(), this->s));
        if (dist_beh < Ego::MIN_VEHICLE_FOLLOWING_DISTANCE) {
            cout << "Beh: Abort prep state: " << (int) state << " with dist_beh: " << dist_beh << endl;
            return prepTraj;
        }
    }
*/
    double prep_lane_speed = this->targetVel;
    Car &car_ah = this->cars_ahead[prep_for_lane];
    Car &car_beh = this->cars_behind[prep_for_lane];

    if (car_ah.getId() >= 0 || car_beh.getId() >= 0) {
        //TODO account for veh behind?
        vector<double> kinematics = get_kinematics(prep_for_lane);
        double new_s = kinematics[0];
        prep_lane_speed = kinematics[1];
        double new_a = kinematics[2];
    }

    // TODO
    /*double cur_lane_speed = this->lane_speeds[this->lane];
    cout << ">>>>>>>>> Prep change to lane: " << prep_for_lane << " (prep ln speed: " << prep_lane_speed << ", cur ln sp: " << cur_lane_speed << ") <<<<<<<<<" << endl;
    cout << ">>>>>>>>> Ego new speed: " << prep_ego_speed << ", v_ah: " << this->cars_ahead[prep_for_lane].getVel() << " (id: " << this->cars_ahead[prep_for_lane].getId()
    << "), v_beh: " << cars_behind[prep_for_lane].getVel() << " (id: " << cars_behind[prep_for_lane].getId() << ")" << endl;
*/
    prepTraj.emplace_back(state, prep_for_lane, (double) Ego::PATH_HORIZON, Road::LANE_WIDTH*this->lane + Road::LANE_WIDTH/2., prep_lane_speed);
    return prepTraj;
}


// Gets next timestep kinematics (position, velocity, acceleration)
//   for a given lane. Tries to choose the maximum velocity and acceleration,
//   given other vehicle positions and accel/velocity constraints.
vector<double> Ego::get_kinematics(Road::LANE ln) {
    assert(ln == Road::LANE::RIGHT_LANE || ln == Road::LANE::MIDDLE_LANE || ln == Road::LANE::LEFT_LANE );

    double new_position;
    double new_velocity;
    double new_accel;
    double max_vel_change = Ego::MAX_ACCEL * DT;
    double max_new_velocity = this->targetVel + max_vel_change;
    Car& vehicle_ahead = this->cars_ahead[ln];

/*    for (auto& c : this->cars_ahead) {
        cout << "Ahd2: cid: " << c.getId() << ", c.s:" << c.getS() << ", ego.s: " << this->s << ", c.addr: " << &c << ", v.addr: " << &vehicle_ahead << ", ln: " << ln << endl;
        if (&c == &vehicle_ahead)
            cout << "right address";
    }
    for (auto& c : this->cars_behind) {
        cout << "Behd2: cid: " << c.getId() << ", c.s:" << c.getS() << ", ego.s: " << this->s << ", c.addr: " << &c << endl;
        if (&c == &vehicle_ahead)
            cout << "wrong address";
    }*/

    if (vehicle_ahead.getId() >= 0) {  // if there's a car ahead
        double car_dist = Ego::frenetCircSDistanceBetweenCars(vehicle_ahead.getS(), this->s);
        assert(car_dist >= 0);

        double buffer_dist = car_dist - Ego::PREFERRED_VEHICLE_FOLLOWING_DISTANCE;

        cout << "vel_ahead: " << vehicle_ahead.getVel() << ", ego.tag_vel: " << this->targetVel << ", ego.spd: " << this->vel << ", dist_ah: " << car_dist << ", buff_dist: " << buffer_dist << endl;
        cout << "ln: " << ln << ", ego.ln: " << this->lane << endl;

        double vel_diff = this->targetVel - vehicle_ahead.getVel();
        if (buffer_dist > 0) {
            if (vel_diff > 0) {
                double estim_decel = vel_diff*vel_diff / buffer_dist;
                double decel = std::min(estim_decel, (double) Ego::MAX_ACCEL);
                new_velocity = this->targetVel - decel * DT;

                cout << "EC_buf_gt_0: " << new_velocity << ", ego targ vel: " << this->targetVel << ", est_decel: " << estim_decel << ", decel: " << decel << endl;
            }
            else {
                new_velocity = max_new_velocity;

                cout << "CE_buf_gt_0: " << new_velocity << ", " << this->targetVel << endl;
            }
        }
        else {
            if (vel_diff > 0) {
                new_velocity = this->targetVel - max_vel_change;
                cout << "EC_buf_less_0: " << new_velocity << ", " << this->targetVel << endl;
            }
            else {
                new_velocity = abs(vel_diff) > max_vel_change ? this->targetVel : vehicle_ahead.getVel() - max_vel_change;
                cout << "CE_buf_less_0: new_vel: " << new_velocity << ", targ.vel: " << this->targetVel << ", ego.spd: " << this->vel << endl;
            }
        }
    } else {
        new_velocity = std::min(max_new_velocity, (double)Road::SPEED_LIMIT_MPS);
    }

    if (new_velocity < 0) {
        cout << "#########################################################" << endl;
        cout << "negative vel: " << new_velocity << " is illegal." << endl;
        cout << "#########################################################" << endl;
        new_velocity = 0;
    }

    double vel_change = new_velocity - this->targetVel;
    new_accel = vel_change / DT; // Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity*DT + new_accel*DT*DT / 2.0;

    if (this->vel > 50) {
        cout << "#########################################################" << endl;
        cout << "Speed exceeds 50 mph limit. Vel: " << this->vel << endl;
        cout << "#########################################################" << endl;
    }

    if (abs(new_accel) - Ego::MAX_ACCEL > 0.0001) {
        cout << "#########################################################" << endl;
        cout << "new accel: " << new_accel << " exceeds limit." << " newvel: " << new_velocity << " targVel: " << this->targetVel << ", egoVel: " << this->vel << ", diff: " << (new_accel - MAX_ACCEL) << ", vel_change: " << vel_change << endl;
        cout << "#########################################################" << endl;
    }

    return {new_position, new_velocity, new_accel};
}

// Returns a true if a vehicle is found behind the current vehicle, false
//   otherwise. The passed reference rVehicle is updated if a vehicle is found.
bool Ego::get_vehicle_behind(Road::LANE ln, Car &rVehicle) {
    double max_s = -1;
    bool found_vehicle = false;
    Car temp_vehicle;
    rVehicle = temp_vehicle;
    for (auto & oc : this->other_cars) {
        temp_vehicle = oc.second;
        if (temp_vehicle.getLane() == ln && Ego::frenetCircSDistanceBetweenCars(temp_vehicle.getS(), this->s) < 0
            && Ego::frenetCircSDistanceBetweenCars(temp_vehicle.getS(), max_s) > 0) {
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
    for (auto & traj : this->other_cars) {
        temp_vehicle = traj.second;
        if (temp_vehicle.getLane() == ln && Ego::frenetCircSDistanceBetweenCars(temp_vehicle.getS(), this->s) >= 0
            && (min_s < 0 || Ego::frenetCircSDistanceBetweenCars(temp_vehicle.getS(), min_s) < 0)) {
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
                this->lane_speeds[i] = (this->targetVel > v_behind &&
                                        abs(Ego::frenetCircSDistanceBetweenCars(this->s, this->cars_behind[i].getS())) > 3 * Ego::IN_LANE_COLLISION_DISTANCE
                                    ? Road::SPEED_LIMIT_MPS: v_behind);
        else {
            // there are vehicles ahead only or both ahead and behind
            this->lane_speeds[i] = v_ahead;
        }

        //TODO  cout << ", lsp: ";
        cout << this->lane_speeds[i] << "||";
    }

    cout << endl << "========================" << endl;
}

void Ego::set_cars_ahead_and_behind() {
    for (int i = 0; i < Road::NUM_LANES; ++i) {
        Car car_ahd;
        this->get_vehicle_ahead((Road::LANE) i, car_ahd);
        this->cars_ahead[i] = car_ahd;

        if (car_ahd.getId() >= 0)
            cout << "Ahead: Lane: " << car_ahd.getLane() << ", id: " << car_ahd.getId() << ", s: " << car_ahd.getS() << ", ego.s: " << this->s << endl;

        Car car_beh;
        this->get_vehicle_behind((Road::LANE) i, car_beh);
        this->cars_behind[i] = car_beh;

        if (car_beh.getId() >= 0)
            cout << "Behind: Lane: " << car_beh.getLane() << ", id: " << car_beh.getId() << ", s: " << car_beh.getS() << ", ego.s: " << this->s << endl;
    }
}

// Calculate circular distance between cars.
// Account for the cars on the different sides of MAX_S.  After MAX_S, the simulator returns 0.
// if car1 is ahead of car2, return distance > 0, otherwise < 0
double Ego::frenetCircSDistanceBetweenCars(const double car1_s, const double car2_s) {
    double circ_dist1 = car1_s - car2_s;
    double circ_dist2 = Road::MAX_S - abs(circ_dist1);
    return abs(circ_dist1) < circ_dist2 ? circ_dist1 : (circ_dist1 < 0 ? circ_dist2 : -1*circ_dist2);
}

double
Ego::calculate_traj_path_cost(const Ego::Trajectory &trajectory, map<int, vector<Car>> &other_car_predictions, Ego::Path &traj_path) {
    traj_path = getPath(trajectory);
    double cost = 0.0;

    auto& costType_weight_map = COST_WEIGHT_LIST.at(trajectory.state);
    for (auto& ctw : costType_weight_map) {
        Ego::COST_TYPES ct = ctw.first;
        double weight = ctw.second;
        double new_cost = weight * COST_FUNC_MAP.at(ct)(*this, trajectory, traj_path, other_car_predictions);
        cost += new_cost;
    }

    return cost;
}

double Ego::inefficiency_cost(const Ego::Trajectory &trajectory, const Ego::Path &traj_path,
                              const map<int, vector<Car>> &other_car_predictions) {
    double maxSpeed = -1;
    int maxSpeedLane = -1;
    for (int i=0; i < Road::NUM_LANES; ++i) {
        if (this->lane_speeds[i] > maxSpeed) {
            maxSpeed = this->lane_speeds[i];
            maxSpeedLane = i;
        }
    }

    assert(maxSpeed > 0);

    double cost = 0;
    if (maxSpeed - this->lane_speeds[this->lane] > Ego::LANE_CHANGE_TRIGGER_VELOCITY_DIFF_MS) {
        cost = (double) abs(maxSpeedLane - (int) trajectory.target_lane) / (Road::NUM_LANES - 1.);
        //TODO vel ineff cost *= fabs(maxSpeed - this->targetVel);
    }

    cout << "inefficiency_cost: " << cost << endl;
    return cost;
}

// 1 - collision. 0 - distance is greater than min
double Ego::collision_and_proximity_cost(const Ego::Trajectory &trajectory, const Ego::Path &ego_path,
                                         const map<int, vector<Car>> &other_car_predictions) {
    double max_cost = 0;
    for (auto const & pred : other_car_predictions) {
        assert(ego_path.pts_x.size() == pred.second.size() && ego_path.pts_s.size() == ego_path.pts_x.size());

        double cost = 0;
        for (int p = 0; p < ego_path.pts_s.size(); ++p) {
            double es = ego_path.pts_s[p];
            double ed = ego_path.pts_d[p];

            Car car = pred.second[p];
            double cs = car.getS();
            double cd = car.getD();
            auto ego_lane = (Road::LANE)(ed / Road::LANE_WIDTH);
            double delta_s = abs(Ego::frenetCircSDistanceBetweenCars(es, cs));
            double delta_d = abs(ed - cd);

            // s coord proximity cost
            if (delta_s < Ego::IN_LANE_COLLISION_DISTANCE && delta_d < Ego::SIDE_COLLISION_DISTANCE) {
                cout << "collision_and_proximity_cost: 1" << endl;
                return 1.;
            }
            else if (delta_s < Ego::MIN_VEHICLE_FOLLOWING_DISTANCE) {
                bool isLaneChangeState =
                    trajectory.state == Ego::FSM_STATES::LCL || trajectory.state == Ego::FSM_STATES::LCR;
                if (ego_lane == car.getLane() || (isLaneChangeState && trajectory.target_lane == car.getLane()))
                    cost = 1. - delta_s / Ego::MIN_VEHICLE_FOLLOWING_DISTANCE;
            }

            if (cost > max_cost)
                max_cost = cost;
        }

        //cout << "cid: " << pred.first << ", dist s: " << fabs(pred.second[0].getS() - this->s) << ", collision/proximity cost: " << max_cost << endl;
    }

    cout << "collision_and_proximity_cost: " << max_cost << endl;

    return max_cost;
}

// Cost: 0 - 1.  This cost increase forces the car towards LC state.  The closer the car matches the target lane speed.
double Ego::lane_change_readiness_cost(const Ego::Trajectory &trajectory, const Ego::Path &traj_path,
                                 const map<int, vector<Car>> &other_car_predictions) {
    Road::LANE intended_lane = trajectory.target_lane;
    Car &car_ah = this->cars_ahead[intended_lane];
    Car &car_beh = this->cars_behind[intended_lane];

    if (car_ah.getId() < 0 && car_beh.getId() < 0) {
        cout << "lane_change_readiness_cost: 1" << endl;
        // no vehicles ahead or behind
        return 1.;
    }

    double cost = exp(-abs(this->targetVel - this->lane_speeds[intended_lane]));

    cout << "lane_change_readiness_cost: " << cost << endl;

    return cost;
}

// off center cost (0-1) used to drive from LC to KL states and prevent KL->PLC before the car is at the center of
// the intended lane. 1 is the cost near center of intended lane. 0 is over the lane
double Ego::off_center_lane_cost(const Ego::Trajectory &trajectory, const Ego::Path &traj_path,
                             const map<int, vector<Car>> &other_car_predictions) {
    assert(trajectory.target_lane == this->lane);

    double half_lane_width = Road::LANE_WIDTH / 2.;
    double mid_curr_lane = Road::LANE_WIDTH * this->lane + half_lane_width;
    double cost = abs(this->d - mid_curr_lane) / half_lane_width;

    cout << "off_center_lane_cost: " << cost << endl;
    return cost;
}

void Ego::recordIncident() {
    // collision
    for (auto const & oc : this->other_cars) {
        double es = this->s;
        double ed = this->d;

        Car car = oc.second;
        double cs = car.getS();
        double cd = car.getD();
        double delta_s = abs(Ego::frenetCircSDistanceBetweenCars(es, cs));
        double delta_d = abs(ed - cd);

       /*TODO cout << "colln?: id: " << car.getId() << ", ds: " << delta_s << ", dd: " << delta_d << ", c.ln: " << car.getLane()
        << ", eg.s: " << this->s << ", eg.d: " << this->d << ", e.lane: " << this->lane << ", elane: " << ego_lane << endl;*/

       if (delta_s < Ego::IN_LANE_COLLISION_DISTANCE && delta_d < Ego::SIDE_COLLISION_DISTANCE) {
            std::cerr << "COLLISION with car id:" << car.getId() << ".  Exiting.." << endl;
            exit(0);
        }
    }

/*TODO    double dt = DT;
    if (!prevTSet) {
        this->prevTSet = true;
    }
    else
        dt = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->prev_t).count())/1000.;
    this->prev_t = std::chrono::steady_clock::now();*/

    cout << "ACCEL: " << getEstimatedAccel() << endl;
    // accel exceeded
    /*double curr_accel = getEstimatedAccel();
    if (this->prev_vel != 0 && abs(curr_accel) > Ego::MAX_ACCEL) {
        cout << "dt: " << dt << endl;
        std::cerr << "Exceeded accel. limit: vel: " << this->vel << ", prev_vel: " << this->prev_vel << ".  Exiting.." << endl;
        exit(0);    
    }
    
    //jerk exceeded
    if (this->prev_accel != INT_MAX && abs(curr_accel - this->prev_accel)/DT > Ego::MAX_JERK) {
        std::cerr << "Exceeded jerk limit: curr_accel: " << curr_accel << ", prev_acc: " << this->prev_accel << ".  Exiting.." << endl;
        exit(0);
    }*/
}






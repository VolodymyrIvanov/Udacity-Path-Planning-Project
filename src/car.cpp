/*
 * car.cpp
 *
 *  Created on: 15.01.2018
 *      Author: VIvanov
 */

#include "car.h"

Car::Car():Car(0.0, 0.0, 0.0) {
}

Car::Car(double s, double speed, double d) {
	init(s, speed, d);
	this->avg_steps = COSTS_AVERAGE_STEPS;
    this->target_lane = -1;
}

Car::~Car() {
}

void Car::init(double s, double speed, double d) {
    this->s = s;
    this->speed = speed;
    this->d = d;
    this->lane_width = DEFAULT_LANE_WIDTH;
    this->current_lane = get_current_lane();
    this->avg_costs.insert(pair<Car_State, double>(CHANGE_TO_LEFT, 0.0));
    this->avg_costs.insert(pair<Car_State, double>(KEEP_LANE, 0.0));
    this->avg_costs.insert(pair<Car_State, double>(CHANGE_TO_RIGHT, 0.0));
}

void Car::update(double s, double speed, double d) {
    this->s = s;
    this->speed = speed;
    this->d = d;
    this->current_lane = get_current_lane();
    //First update
    if (this->target_lane < 0) {
        this->target_lane = this->current_lane;
    }
}

int Car::get_current_lane() {
    return this->lane_width != 0 ? this->d / this->lane_width : 0;
}

bool Car::is_on_target_lane() {
    return is_on_lane_center(target_lane);
}

bool Car::is_on_lane_center(int lane) {
    double center_line = lane * DEFAULT_LANE_WIDTH + DEFAULT_LANE_WIDTH / 2.0;
    return fabs(d - center_line) < CENTER_LANE_THRESHOLD;
}

void Car::add_costs(map<Car_State, double> costs) {
    for (auto& cost : costs) {
        double avg = avg_costs.at(cost.first);
        avg = (avg * avg_steps) - avg;
        avg += cost.second;
        avg /= avg_steps;
        avg_costs.at(cost.first) = avg;
    }
}

Car_State Car::get_min_cost_state() {
    pair<Car_State, double> min = *min_element(avg_costs.begin(), avg_costs.end(), &Car::compare_cost);
    return min.first;
}

bool Car::compare_cost(pair<Car_State, double> c1, pair<Car_State, double> c2) {
    return c1.second < c2.second;
}

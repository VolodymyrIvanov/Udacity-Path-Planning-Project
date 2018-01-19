/*
 * car.h
 *
 *  Created on: 15.01.2018
 *      Author: VIvanov
 */

#ifndef CAR_H_
#define CAR_H_

#include <vector>
#include <map>
#include <math.h>
#include <algorithm>
#include "consts.h"

using namespace std;

class Car {
public:
    double s;
    double speed;
    double d;
    map<Car_State, double> avg_costs;
    int avg_steps;
    double lane_width;
    int current_lane;
    int target_lane;
    /** Constructors */
    Car();
    Car(double s, double speed, double d);

    virtual ~Car();

    void update(double s, double speed, double d);
    void init(double s, double speed, double d);
    int get_current_lane();
    bool is_on_target_lane();
    double get_long_distance() { return s; };
    double get_speed() { return speed; };
    double get_lat_distance() { return d; };

    double get_distance_to_car(Car& other) { return fabs(get_long_distance() - other.get_long_distance()); };
    void add_costs(map<Car_State, double> costs);
    void clear_costs() { for (auto& it : avg_costs) it.second = 0.0; };
    Car_State get_min_cost_state();
private:
    bool is_on_lane_center(int lane);
    static bool compare_cost(pair<Car_State, double> c1, pair<Car_State, double> c2);
};

#endif /* CAR_H_ */

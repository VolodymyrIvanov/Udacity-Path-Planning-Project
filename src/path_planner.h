/*
 * path_planner.h
 *
 *  Created on: 18.01.2018
 *      Author: VIvanov
 */

#ifndef SRC_PATH_PLANNER_H_
#define SRC_PATH_PLANNER_H_

#include <vector>
#include <map>
#include "consts.h"
#include "car.h"

using namespace std;

class PathPlanner {
public:
    int laneCount;
    int collision_distance;
    int free_distance;
    double recommended_speed;
    double speed_limit;
    map<int, vector<Car>> other_cars;

    PathPlanner(int laneCount);
    virtual ~PathPlanner();

    void load_sensor_data(vector<vector<double>> sensor_fusion);

    /**
     * General rules of behavior:
     * - car should drive as on right lane as possible (even if no obstacles in current (not most right) lane)
     * - car passing from right side is forbidden
     */
    int next_lane(Car& car);
    Car* get_closest_car(Car& car, Direction direction) { return get_closest_car(car, car.current_lane, direction); };
    Car* get_closest_car(Car& car, int lane, Direction direction);

private:
    void clear_cars() { for (auto& it : other_cars) it.second.clear(); };
    void evaluate_costs(Car& car);
    bool is_left_most_lane(int lane) { return lane <= 0; };
    bool is_right_most_lane(int lane) { return lane >= (laneCount - 1); };
    bool is_neighbor_lane_on_left(int lane) { return lane > 0; };
    bool is_neighbor_lane_on_right(int lane) { return lane < (laneCount - 1); };
};

#endif /* SRC_PATH_PLANNER_H_ */

/*
 * path_planner.cpp
 *
 *  Created on: 18.01.2018
 *      Author: VIvanov
 */

#include <iostream>
#include "path_planner.h"

using namespace std;

PathPlanner::PathPlanner(int laneCount) {
    this->laneCount = laneCount;
    this->collision_distance = COLLISION_DISTANCE;
    this->free_distance = FREE_DISTANCE_TO_CHANGE_LANE;
    this->speed_limit = SPEED_LIMIT_MPH - SPEED_LIMIT_THRESHOLD;
    this->recommended_speed = this->speed_limit;

    for (int i = 0; i < laneCount; i++) {
        vector<Car> cars;
        other_cars.insert(pair<int, vector<Car>>(i, cars));
    }
}

PathPlanner::~PathPlanner() {
}

void PathPlanner::load_sensor_data(vector<vector<double>> sensor_fusion) {
    //First - clear previous data
    clear_cars();

    //Load sensor data into map of cars based on lane as a key
    for (int i = 0; i < sensor_fusion.size(); i++) {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double car_s = sensor_fusion[i][5];
        double car_d = sensor_fusion[i][6];
        double car_speed = sqrt(vx * vx + vy * vy);
        Car other_car = Car(car_s, car_speed, car_d);
        const int car_lane = other_car.current_lane;
        map<int, vector<Car>>::iterator it = other_cars.find(car_lane);
        if (it != other_cars.end()) {
            it->second.push_back(other_car);
        }
    }
}

int PathPlanner::next_lane(Car& car) {

    int lane = car.current_lane;
    int new_lane;

    //Check closest car ahead in the same lane
    Car* closest_car = get_closest_car(car, FORWARD);

    if (closest_car && car.get_distance_to_car(*closest_car) < collision_distance) {
        //Here is the car in the same lane and distance is less than critical
        //cout << "Collision with car speed " << closest_car->speed << endl;
        recommended_speed = closest_car->speed;
    }
    else {
        recommended_speed = speed_limit;
    }

    //Make possible to change lane only after some acceleration
    if (car.get_speed() > MIN_SPEED_FOR_LANE_CHANGE) {
        //Evaluate state costs for car
        evaluate_costs(car);

        //for (auto&cost : car.avg_costs) {
        //    cout << "cost [" << cost.first << "]=" << cost.second << endl;
        //}

        //Find the state with minimal cost
        Car_State state = car.get_min_cost_state();
        switch (state) {
            case CHANGE_TO_LEFT:
                new_lane = lane - 1;
                //cout << "CHANGE_TO_LEFT: new lane is " << new_lane << endl;
                //Change lane - no limitation of speed more
                recommended_speed = speed_limit;
                break;

            case CHANGE_TO_RIGHT:
                new_lane = lane + 1;
                //cout << "CHANGE_TO_RIGHT: new lane is " << new_lane << endl;
                //Change lane - no limitation of speed more
                recommended_speed = speed_limit;
                break;

            case KEEP_LANE:
            default:
                new_lane = lane;
        }
        car.target_lane = new_lane;
        return new_lane;
    }
    else {
        return lane;
    }
}

Car* PathPlanner::get_closest_car(Car& car, int lane, Direction direction) {

    Car* candidate = 0;

    map<int, vector<Car>>::iterator it = other_cars.find(lane);

    if (it != other_cars.end()) {

        vector<Car> cars = it->second;
        double closest_car_dist = 1000000;

        for (auto& other_car : cars) {

            double other_car_s = other_car.s;
            double dist = (FORWARD == direction) ? other_car_s - car.s : car.s - other_car_s;

            //Based on direction (distance sign - must be only positive)
            if (dist >= 0 && dist < closest_car_dist) {
                closest_car_dist = dist;
                candidate = &other_car;
            }
        }
    }

    return candidate;
}

void PathPlanner::evaluate_costs(Car& car) {

    map<Car_State, double> costs;

    int lane = car.current_lane;
    if (is_left_most_lane(lane)) {
        //Left Most lane. Possible states - keep lane or change to right
        //Keep lane - subtract cost
        costs.insert(pair<Car_State, double>(KEEP_LANE, -0.5));
        //Change to right - subtract cost more, because it is even more preferable (general rule - car must drive as right as possible)
        costs.insert(pair<Car_State, double>(CHANGE_TO_RIGHT, is_neighbor_lane_on_right(lane) ? -0.6 : 500.0));
        //Change to left is impossible - add big cost
        costs.insert(pair<Car_State, double>(CHANGE_TO_LEFT, 500.0));

        //Don't analyze distances to cars in the same lane, because even if the lane is busy, passing from right side is forbidden

        //Analyze distances to cars in the right neighbor lane
        if (is_neighbor_lane_on_right(lane)) {

            Car* car_in_front = get_closest_car(car, lane + 1, FORWARD);
            Car* car_in_back = get_closest_car(car, lane + 1, BACKWARD);

            if (car_in_front == 0 && car_in_back == 0) {
                //No cars - decrease cost
                costs.at(CHANGE_TO_RIGHT) -= 10;
                //cout << "LEFT: No cars on right" << endl;
            }
            else {
                if (car_in_front && car.get_distance_to_car(*car_in_front) < free_distance) {
                    //Not enough free space - increase cost
                    costs.at(CHANGE_TO_RIGHT) += 5;
                    //cout << "Can't change to right: Car in front" << endl;
                }
                if (car_in_back && car.get_distance_to_car(*car_in_back) < free_distance) {
                    //Not enough free space - increase cost
                    costs.at(CHANGE_TO_RIGHT) += 5;
                    //cout << "Can't change to right: Car back" << endl;
                }
            }
        }
        car.add_costs(costs);
    }
    else if (is_right_most_lane(lane)) {
        //Right Most lane. Possible states - keep lane or change to left
        //Keep lane - subtract cost
        costs.insert(pair<Car_State, double>(KEEP_LANE, -0.5));
        costs.insert(pair<Car_State, double>(CHANGE_TO_LEFT, is_neighbor_lane_on_left(lane) ? 0.0 : 500.0));
        //Change to right is impossible - add big cost
        costs.insert(pair<Car_State, double>(CHANGE_TO_RIGHT, 500.0));

        //Analyze distances to cars in the same lane
        Car* car_in_front = get_closest_car(car, FORWARD);
        if (car_in_front && car.get_distance_to_car(*car_in_front) < collision_distance) {
            //Collision - increase cost
            costs.at(KEEP_LANE) += 1;
            //cout << "Collision" << endl;

            //Only in this case - possible change to left
            //Analyze distances to cars in the left neighbor lane
            if (is_neighbor_lane_on_left(lane)) {

                Car* car_in_front = get_closest_car(car, lane - 1, FORWARD);
                Car* car_in_back = get_closest_car(car, lane - 1, BACKWARD);

                if (car_in_front == 0 && car_in_back == 0) {
                    //No cars - decrease cost
                    costs.at(CHANGE_TO_LEFT) -= 10;
                }
                else {
                    if (car_in_front && car.get_distance_to_car(*car_in_front) < free_distance) {
                        //Not enough free space - increase cost
                        costs.at(CHANGE_TO_LEFT) += 5;
                        //cout << "Can't change to left: Car in front" << endl;
                    }
                    if (car_in_back && car.get_distance_to_car(*car_in_back) < free_distance) {
                        //Not enough free space - increase cost
                        costs.at(CHANGE_TO_LEFT) += 5;
                        //cout << "Can't change to left: Car back" << endl;
                    }
                }
            }
        }

        car.add_costs(costs);
    }
    else {
        //Middle lanes. Possible states - keep lane or change to left or right
        //Keep lane - subtract cost
        costs.insert(pair<Car_State, double>(KEEP_LANE, -0.5));
        costs.insert(pair<Car_State, double>(CHANGE_TO_LEFT, is_neighbor_lane_on_left(lane) ? 0.0 : 500.0));
        costs.insert(pair<Car_State, double>(CHANGE_TO_RIGHT, is_neighbor_lane_on_right(lane) ? 0.0 : 500.0));

        //Analyze distances to cars in the same lane
        Car* car_in_front = get_closest_car(car, FORWARD);
        if (car_in_front && car.get_distance_to_car(*car_in_front) < collision_distance) {
            //Collision - increase cost
            costs.at(KEEP_LANE) += 1;
            //cout << "Collision" << endl;

            //Only in this case - possible change to left
            //Analyze distances to cars in the left neighbor lane
            if (is_neighbor_lane_on_left(lane)) {

                Car* car_in_front = get_closest_car(car, lane - 1, FORWARD);
                Car* car_in_back = get_closest_car(car, lane - 1, BACKWARD);

                if (car_in_front == 0 && car_in_back == 0) {
                    //No cars - decrease cost
                    costs.at(CHANGE_TO_LEFT) -= 10;
                }
                else {
                    if (car_in_front && car.get_distance_to_car(*car_in_front) < free_distance) {
                        //Not enough free space - increase cost
                        costs.at(CHANGE_TO_LEFT) += 5;
                        //cout << "Can't change to left: Car in front" << endl;
                    }
                    if (car_in_back && car.get_distance_to_car(*car_in_back) < free_distance) {
                        //Not enough free space - increase cost
                        costs.at(CHANGE_TO_LEFT) += 5;
                        //cout << "Can't change to left: Car back" << endl;
                    }
                }
            }
        }

        //Analyze distances to cars in the right neighbor lane
        if (is_neighbor_lane_on_right(lane)) {

            Car* car_in_front = get_closest_car(car, lane + 1, FORWARD);
            Car* car_in_back = get_closest_car(car, lane + 1, BACKWARD);

            if (car_in_front == 0 && car_in_back == 0) {
                //No cars - decrease cost
                costs.at(CHANGE_TO_RIGHT) -= 10;
            }
            else {
                if (car_in_front && car.get_distance_to_car(*car_in_front) < free_distance) {
                    //Not enough free space - increase cost
                    costs.at(CHANGE_TO_RIGHT) += 5;
                    //cout << "Can't change to right: Car in front" << endl;
                }
                if (car_in_back && car.get_distance_to_car(*car_in_back) < free_distance) {
                    //Not enough free space - increase cost
                    costs.at(CHANGE_TO_RIGHT) += 5;
                    //cout << "Can't change to right: Car back" << endl;
                }
            }
        }

        car.add_costs(costs);
    }
}

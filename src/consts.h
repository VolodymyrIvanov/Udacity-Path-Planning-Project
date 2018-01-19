/*
 * consts.h
 *
 *  Created on: 15.01.2018
 *      Author: VIvanov
 */

#ifndef SRC_CONSTS_H_
#define SRC_CONSTS_H_

#define DEFAULT_LANE_WIDTH 4.0
#define COLLISION_DISTANCE 20
#define FREE_DISTANCE_TO_CHANGE_LANE 100
#define MIN_SPEED_FOR_LANE_CHANGE 15.0
#define SPEED_LIMIT_MPH 50.0
#define SPEED_LIMIT_THRESHOLD 0.5
#define COSTS_AVERAGE_STEPS 10
#define NEXT_PATH_SIZE 150
#define MAX_ACCELERATION 10 * 0.02 * 0.8
#define CENTER_LANE_THRESHOLD 0.6

enum Direction {
	FORWARD,
	BACKWARD
};

enum Car_State {
    CHANGE_TO_LEFT,
    KEEP_LANE,
    CHANGE_TO_RIGHT };

#endif /* SRC_CONSTS_H_ */

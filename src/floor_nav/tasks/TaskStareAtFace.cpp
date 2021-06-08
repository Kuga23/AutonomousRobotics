#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO
/*
TaskIndicator TaskStareAtFace::initialise() 
{
    ROS_INFO("Setting heading to %.2f deg", cfg.target*180./M_PI);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        initial_heading = tpose.theta;
    } else {
        initial_heading = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}*/

TaskIndicator TaskStareAtFace::iterate()
{
	if(env->facedetected){
		if (fabs(env->diff) < cfg.pixel_threshold) {
			return TaskStatus::TASK_COMPLETED;
		}
		double rot = cfg.K*env->diff;
		if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
		if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
		env->publishVelocity(0.0, rot);
	}
	/*else{
		// no face 
		return TaskStatus::TASK_COMPLETED;
	}*/
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);

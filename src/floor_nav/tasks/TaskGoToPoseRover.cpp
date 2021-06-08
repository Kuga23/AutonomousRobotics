#include <math.h>
#include "TaskGoToPoseRover.h"
#include "floor_nav/TaskGoToPoseRoverConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskGoToPoseRover::initialise() 
{
    ROS_INFO("Going to %.2f %.2f with heading %.2f",cfg.goal_x,cfg.goal_y, cfg.goal_theta);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}
double dist(double a, double b){
		return std::min(fabs(a-b), fabs(2*M_PI-(a-b)));
	}

TaskIndicator TaskGoToPoseRover::iterate()
{	
	
	
	const geometry_msgs::Pose2D & tpose = env->getPose2D();
	double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
	
	double beta =remainder(theta_init + cfg.goal_theta-tpose.theta,2*M_PI);
	
	ROS_INFO("Going to (%f, %f, %.2f) currently at heading (%f, %f, %.2f), beta %f", cfg.goal_x, cfg.goal_y, cfg.goal_theta,tpose.x-x_init, tpose.y-y_init, tpose.theta-theta_init, beta);

	double vx = cfg.k_x * (cfg.goal_x+x_init-tpose.x);
	double vy = cfg.k_y * (cfg.goal_y+y_init-tpose.y);
	double vel_x = std::max(std::min(vx*cos(tpose.theta)+vy*sin(tpose.theta),cfg.max_velocity),-cfg.max_velocity);
	double vel_y = std::max(std::min(-vx*sin(tpose.theta)+vy*cos(tpose.theta),cfg.max_velocity),-cfg.max_velocity);

	double rot = std::max(std::min(cfg.k_beta*beta,cfg.max_angular_velocity),-cfg.max_angular_velocity);
    ROS_INFO("command global (%f, %f, %f), robot (%f, %f, %f)", vx, vy, rot , vel_x, vel_y, rot);

	#ifdef DEBUG_GOTO
		printf("Cmd v %.2f r %.2f\n",vel,rot);
	#endif
	env->publishVelocity(vel_x, vel_y, rot);
	
	if (r < cfg.dist_threshold && fabs(beta) < cfg.angle_threshold) {
		return TaskStatus::TASK_COMPLETED;
	}
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPoseRover::terminate()
{
    env->publishVelocity(0,0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPoseRover);

#include <math.h>
#include "TaskGoToPose.h"
#include "floor_nav/TaskGoToPoseConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskGoToPose::initialise() 
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


TaskIndicator TaskGoToPose::iterate()
{	
	if (cfg.smart) {
	
		const geometry_msgs::Pose2D & tpose = env->getPose2D();
		double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
		
		double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
		double beta = remainder(theta_init + cfg.goal_theta-tpose.theta,2*M_PI);
		
		/*ROS_INFO("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
				tpose.x, tpose.y, tpose.theta*180./M_PI,
				cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);*/
		ROS_INFO("Going to %.2f currently at heading %.2f",cfg.goal_theta, tpose.theta);

		if (fabs(alpha) > M_PI/9) {
				double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
		#ifdef DEBUG_GOTO
				printf("Cmd v %.2f r %.2f\n",0.,rot);
		#endif
				env->publishVelocity(0,rot);
		} else {
			
			
			double vel = cfg.k_v * r;
			double rot = std::max(std::min(cfg.k_alpha*alpha+cfg.k_beta*beta,cfg.max_angular_velocity),-cfg.max_angular_velocity);
			if (vel > cfg.max_velocity) vel = cfg.max_velocity;
		#ifdef DEBUG_GOTO
			printf("Cmd v %.2f r %.2f\n",vel,rot);
		#endif
			env->publishVelocity(vel, rot);
		}
		if (r < cfg.dist_threshold && fabs(beta) < cfg.angle_threshold) {
			return TaskStatus::TASK_COMPLETED;
		}
		return TaskStatus::TASK_RUNNING;
	
	} else {
		const geometry_msgs::Pose2D & tpose = env->getPose2D();
		double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
		
		if (r < cfg.dist_threshold) {
			double beta = remainder(-tpose.theta +theta_init + cfg.goal_theta,2*M_PI);
			double rot = ((beta <0)?+1:-1)*cfg.max_angular_velocity;
			
			if (fabs(beta) < cfg.angle_threshold) {
			return TaskStatus::TASK_COMPLETED;}
			env->publishVelocity(0,rot);
		
	    }
		double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI); 		
	#ifdef DEBUG_GOTO
		printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
				tpose.x, tpose.y, tpose.theta*180./M_PI,
				cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
	#endif
		if (fabs(alpha) > M_PI/9) {
			double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
	#ifdef DEBUG_GOTO
			printf("Cmd v %.2f r %.2f\n",0.,rot);
	#endif	
			env->publishVelocity(0,rot);
		} else {
			double vel = cfg.k_v * r;
			double rot = std::max(std::min(cfg.k_alpha*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);
			if (vel > cfg.max_velocity) vel = cfg.max_velocity;
	#ifdef DEBUG_GOTO
			printf("Cmd v %.2f r %.2f\n",vel,rot);
	#endif
			env->publishVelocity(vel, rot);
		}
		return TaskStatus::TASK_RUNNING;	
	}
	

}

TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);

#ifndef TASK_GOTO_POSE_ROVER_H
#define TASK_GOTO_POSE_ROVER_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskGoToPoseRoverConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskGoToPoseRover : public TaskInstance<TaskGoToPoseRoverConfig,SimTasksEnv>
    {
        protected:
            double x_init,y_init,theta_init;
        public:
            TaskGoToPoseRover(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskGoToPoseRover() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryGoToPoseRover : public TaskDefinition<TaskGoToPoseRoverConfig, SimTasksEnv, TaskGoToPoseRover>
    {

        public:
            TaskFactoryGoToPoseRover(TaskEnvironmentPtr env) : 
                Parent("GoToPoseRover","Reach a desired destination and a given pose",true,env) {}
            virtual ~TaskFactoryGoToPoseRover() {};
    };
};

#endif // TASK_GOTO_POSE_ROVER_H

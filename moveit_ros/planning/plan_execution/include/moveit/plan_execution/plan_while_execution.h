#ifndef MOVEIT_PLAN_EXECUTION_PLAN_WHILE_EXECUTION_
#define MOVEIT_PLAN_EXECUTION_PLAN_WHILE_EXECUTION_

#include <queue>

#include <moveit/macros/class_forward.h>
#include <moveit/plan_execution/plan_representation.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/trajectory_monitor.h>
#include <moveit/sensor_manager/sensor_manager.h>
#include <pluginlib/class_loader.hpp>

#include "moveit/plan_execution/plan_execution.h"

/** \brief This namespace includes functionality specific to the execution and monitoring of motion plans */
namespace plan_execution
{
MOVEIT_CLASS_FORWARD(PlanWhileExecution);

class PlanWhileExecution : public PlanExecution
{
public:
    PlanWhileExecution(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                       const trajectory_execution_manager::TrajectoryExecutionManagerPtr& trajectory_execution);
    ~PlanWhileExecution();

    void planWhileExecute(ExecutableMotionPlan& plan, const Options& opt);
    bool executeAndMonitorAsync(ExecutableMotionPlan& plan);
    void executeAndMonitorThread();

    void stop();

private:
    bool started_ = false;
    bool plan_finished_ = true;
    std::queue<ExecutableMotionPlan> plan_queue_;
    std::unique_ptr<boost::thread> execution_thread_;

    boost::mutex mutex_;
    boost::condition_variable cond_;
    void add_plan_to_q(ExecutableMotionPlan& plan);
    bool get_plan_from_q(ExecutableMotionPlan& plan);
};
}  // namespace plan_execution
#endif
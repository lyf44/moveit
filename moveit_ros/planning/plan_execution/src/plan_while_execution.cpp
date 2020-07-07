#include "moveit/plan_execution/plan_while_execution.h"

namespace plan_execution
{

PlanWhileExecution::PlanWhileExecution(
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
    const trajectory_execution_manager::TrajectoryExecutionManagerPtr& trajectory_execution)
  : PlanExecution(planning_scene_monitor, trajectory_execution)
{
}

PlanWhileExecution::~PlanWhileExecution()
{
}

void PlanWhileExecution::planWhileExecute(ExecutableMotionPlan& plan, const Options& opt)
{
    plan.planning_scene_monitor_ = planning_scene_monitor_;
    plan.planning_scene_ = planning_scene_monitor_->getPlanningScene();

    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        plan_finished_ = false;
    }

    // perform initial configuration steps & various checks
    preempt_requested_ = false;
    unsigned int waypoint_num = 0;
    unsigned int execution_num = 1;
    bool goal_reached = false;

    // run the actual motion plan & execution
    unsigned int max_replan_attempts =
        opt.replan_ ? (opt.replan_attempts_ > 0 ? opt.replan_attempts_ : default_max_replan_attempts_) : 1;
    unsigned int replan_attempts = 0;
    bool previously_solved = false;

    // run a planning loop
    do {
        waypoint_num++;
        if (waypoint_num > 2) {
            goal_reached = true;
            break;
        }
        ROS_INFO_NAMED("planWhileExecute", "Planning waypoint %u", waypoint_num);

        bool solved = false;
        replan_attempts = 0;
        do {
            replan_attempts++;
            ROS_INFO_NAMED("planWhileExecute", "Planning attempt %u of at most %u", replan_attempts, max_replan_attempts);
            if (opt.before_plan_callback_)
                opt.before_plan_callback_();

            new_scene_update_ = false;  // we clear any scene updates to be evaluated because we are about to compute a new
                                        // plan, which should consider most recent updates already

            if (preempt_requested_)
                break;

            // if we never had a solved plan, or there is no specified way of fixing plans, just call the planner; otherwise,
            // try to repair the plan we previously had;
            solved = (!previously_solved || !opt.repair_plan_callback_) ?
                        opt.plan_callback_(plan) :
                        opt.repair_plan_callback_(plan, trajectory_execution_manager_->getCurrentExpectedTrajectoryIndex());

            if (preempt_requested_)
                break;

            // if planning fails in a manner that is not recoverable, we exit the loop,
            // otherwise, we attempt to continue, if replanning attempts are left
            if (plan.error_code_.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED ||
                plan.error_code_.val == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN ||
                plan.error_code_.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA) {
                if (plan.error_code_.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA &&
                    opt.replan_delay_ > 0.0) {
                    ros::WallDuration d(opt.replan_delay_);
                    d.sleep();
                }
                continue;
            }

            // abort if no plan was found
            if (solved) {
                previously_solved = true;
                break;
            }
            else
                break;

        } while (!preempt_requested_ && max_replan_attempts > replan_attempts);

        if (plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            if (opt.before_execution_callback_)
                opt.before_execution_callback_();

            if (preempt_requested_)
                break;

            // execute the trajectory, and monitor its executionm
            executeAndMonitorAsync(plan);
        }

        // TODO determine whether goal is reached???

    } while (!goal_reached && !preempt_requested_);

    ROS_INFO_NAMED("planWhileExecute", "planning finished");

    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        plan_finished_ = true;
    }

    if (preempt_requested_) {
        ROS_DEBUG_NAMED("plan_execution", "PlanExecution was preempted");
        plan.error_code_.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    }

    if (opt.done_callback_)
        opt.done_callback_();

    if (plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        ROS_DEBUG_NAMED("plan_execution", "PlanExecution finished successfully.");
    else
        ROS_DEBUG_NAMED("plan_execution", "PlanExecution terminating with error code %d - '%s'", plan.error_code_.val,
                        getErrorCodeString(plan.error_code_).c_str());

    // wait for execution thread to finish
    if (started_) {
        execution_thread_->join();
        started_ = false;
        execution_thread_.reset();
    }
    while (!plan_queue_.empty()) {
        plan_queue_.pop();
    }
}

bool PlanWhileExecution::executeAndMonitorAsync(ExecutableMotionPlan& plan)
{
    add_plan_to_q(plan);

    // start a new thread if needed
    if (!started_) {
        ROS_INFO_NAMED("plan_while_execution", "new execution thread created");
        execution_thread_.reset(new boost::thread(&PlanWhileExecution::executeAndMonitorThread, this));
        started_ = true;
    }
    else {
        ROS_INFO_NAMED("plan_while_execution", "adding new plan to existing execution thread");
        boost::lock_guard<boost::mutex> lock(mutex_);
        cond_.notify_one();
    }

    // ExecutableMotionPlan plan_copy = plan;  // make a copy of the plan
    
    return true;
}

void PlanWhileExecution::executeAndMonitorThread()
{
    while (!preempt_requested_) {
        ExecutableMotionPlan plan;
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            bool res = get_plan_from_q(plan);
            if (!res) {
                ROS_WARN_NAMED("executeAndMonitorThread", "no more executable plan in queue");
                if (plan_finished_) break;
                ROS_WARN_NAMED("executeAndMonitorThread", "entering wait");
                cond_.wait(lock); // unlock mutex when enter, lock mutex when exit.
                ROS_WARN_NAMED("executeAndMonitorThread", "waking up");
                continue;
            }
            // mutex is unlocked here as it goes out of scope
        }

        ROS_INFO("executeAndMonitorThread: starting execution!!!");
        trajectory_execution_manager_->setAllowedStartTolerance(0); // dont check start tolerance
        moveit_msgs::MoveItErrorCodes error = executeAndMonitor(plan);
        if (error.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("executeAndMonitorThread: execution succeded");
        }
        else {
            ROS_INFO("executeAndMonitorThread: execution failed");
        }
    }
    ROS_INFO("executeAndMonitorThread: thread exit");
}

void PlanWhileExecution::add_plan_to_q(ExecutableMotionPlan& plan)
{
    plan_queue_.push(plan); // this should make a copy
}

bool PlanWhileExecution::get_plan_from_q(ExecutableMotionPlan& plan)
{
    if (plan_queue_.empty()) {
        return false;
    }

    plan = plan_queue_.front();
    plan_queue_.pop();
    return true;
}

}
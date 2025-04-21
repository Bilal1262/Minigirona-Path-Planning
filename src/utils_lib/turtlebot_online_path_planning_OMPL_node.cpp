#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <vector>
#include <memory>
#include <cmath>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class StateValidityChecker {
public:
    StateValidityChecker(double distance = 0.25, bool is_unknown_valid = true)  // Reduced to 0.15m for flexibility
        : distance_(distance), is_unknown_valid_(is_unknown_valid), there_is_map_(false) {}

    void set(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        if (!msg) {
            ROS_ERROR("Received null occupancy grid message!");
            return;
        }
        map_ = std::vector<int8_t>(msg->data.begin(), msg->data.end());
        resolution_ = msg->info.resolution;
        origin_ = {msg->info.origin.position.x, msg->info.origin.position.y};
        width_ = msg->info.width;
        height_ = msg->info.height;
        there_is_map_ = true;
    }

    bool thereIsMap() const { return there_is_map_; }

    bool isValid(double x, double y) const {
        if (!there_is_map_) {
            return false;
        }
        int map_x, map_y;
        positionToMap({x, y}, map_x, map_y);
        if (map_x < 0 || map_x >= static_cast<int>(width_) || map_y < 0 || map_y >= static_cast<int>(height_)) {
            return is_unknown_valid_;
        }
        int radius_in_cells = static_cast<int>(distance_ / resolution_);
        for (int dx = -radius_in_cells; dx <= radius_in_cells; ++dx) {
            for (int dy = -radius_in_cells; dy <= radius_in_cells; ++dy) {
                int nx = map_x + dx;
                int ny = map_y + dy;
                if (nx < 0 || nx >= static_cast<int>(width_) || ny < 0 || ny >= static_cast<int>(height_)) {
                    if (!is_unknown_valid_) return false;
                } else {
                    size_t index = ny * width_ + nx;
                    if (index >= map_.size()) {
                        ROS_ERROR("Index out of bounds: %zu >= %zu", index, map_.size());
                        return false;
                    }
                    if (map_[index] == 100) return false;  // Standard obstacle threshold
                    if (map_[index] == -1 && !is_unknown_valid_) return false;
                }
            }
        }
        return true;
    }

    bool checkPath(const std::vector<std::pair<double, double>>& path) const {
        double step_size = distance_ / 2.0;  // Simpler, moderate check (0.075m)
        for (size_t i = 1; i < path.size(); ++i) {
            double x0 = path[i - 1].first;
            double y0 = path[i - 1].second;
            double x1 = path[i].first;
            double y1 = path[i].second;
            double dist = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
            int num_steps = std::max(1, static_cast<int>(dist / step_size));
            for (int j = 0; j <= num_steps; ++j) {
                double t = static_cast<double>(j) / num_steps;
                double x = x0 + (x1 - x0) * t;
                double y = y0 + (y1 - y0) * t;
                if (!isValid(x, y)) {
                    return false;
                }
            }
        }
        return true;
    }

    bool findNearestValid(double& x, double& y) const {
        if (isValid(x, y)) return true;
        ROS_INFO("Finding nearest valid position from (%f, %f)", x, y);
        double step = resolution_;
        for (double r = step; r <= 1.0; r += step) {
            for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 8) {
                double nx = x + r * std::cos(theta);
                double ny = y + r * std::sin(theta);
                if (isValid(nx, ny)) {
                    x = nx;
                    y = ny;
                    ROS_INFO("Adjusted to valid position (%f, %f)", x, y);
                    return true;
                }
            }
        }
        ROS_ERROR("No valid position found near (%f, %f)", x, y);
        return false;
    }

private:
    void positionToMap(const std::vector<double>& p, int& map_x, int& map_y) const {
        map_x = static_cast<int>(((p[0] - origin_[0]) / resolution_));
        map_y = static_cast<int>(((p[1] - origin_[1]) / resolution_));
    }

    std::vector<int8_t> map_;
    double resolution_ = 0.0;
    std::vector<double> origin_;
    size_t width_ = 0, height_ = 0;
    bool there_is_map_ = false;
    double distance_;
    bool is_unknown_valid_;
};

class OnlinePlanner {
public:
    OnlinePlanner() : nh_(""), svc_(0.25), dominion_{-40.0, 40.0, -40.0, 40.0} {
        cmdPub_ = nh_.advertise<geometry_msgs::Twist>("/turtlebot/kobuki/commands/velocity", 10);
        markerPub_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 1);
        gridmapSub_ = nh_.subscribe("/projected_map", 1, &OnlinePlanner::gridmapCallback, this);
        odomSub_ = nh_.subscribe("/minigirona/navigator/odometry", 1, &OnlinePlanner::odomCallback, this);
        goalSub_ = nh_.subscribe("/move_base_simple/goal", 1, &OnlinePlanner::goalCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.1), &OnlinePlanner::controllerCallback, this);

        Kv_ = 0.5;
        Kw_ = 0.5;
        vMax_ = 0.15;
        wMax_ = 0.3;
        lastMapTime_ = ros::Time(0);
        currentPose_ = {0.0, 0.0, 0.0};
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!msg) {
            ROS_ERROR("Received null odometry message!");
            return;
        }
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);  // No condition needed
        currentPose_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw};
    }

    void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        if (!msg) {
            ROS_ERROR("Received null occupancy grid message!");
            return;
        }
        if ((msg->header.stamp - lastMapTime_).toSec() > 0.1) {
            lastMapTime_ = msg->header.stamp;
            svc_.set(msg);
            if (!path_.empty() && !svc_.checkPath(path_)) {
                sendCommand(0, 0);
                plan();
            }
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!msg) {
            ROS_ERROR("Received null goal message!");
            return;
        }
        if (svc_.thereIsMap()) {
            double goal_x = msg->pose.position.x;
            double goal_y = msg->pose.position.y;
            if (!svc_.isValid(goal_x, goal_y)) {
                if (!svc_.findNearestValid(goal_x, goal_y)) {
                    ROS_ERROR("Goal position (%f, %f) invalid and no valid nearby position found", goal_x, goal_y);
                    return;
                }
            }
            goal_ = {goal_x, goal_y};
            ROS_INFO("New goal received: (%f, %f)", goal_.first, goal_.second);
            if (currentPose_[0] == 0.0 && currentPose_[1] == 0.0 && currentPose_[2] == 0.0) {
                return;
            }
            plan();
        }
    }

    void plan() {
        path_.clear();
        double start_x = currentPose_[0], start_y = currentPose_[1];
        if (!svc_.isValid(start_x, start_y)) {
            ROS_INFO("Start position (%f, %f) invalid, finding nearest valid...", start_x, start_y);
            if (!svc_.findNearestValid(start_x, start_y)) {
                ROS_ERROR("Cannot find valid start position near (%f, %f)", start_x, start_y);
                return;
            }
            currentPose_[0] = start_x;
            currentPose_[1] = start_y;
            ROS_INFO("Adjusted start to valid position (%f, %f)", start_x, start_y);
        }

        double goal_x = goal_.first, goal_y = goal_.second;
        if (!svc_.isValid(goal_x, goal_y)) {
            ROS_INFO("Goal position (%f, %f) invalid, finding nearest valid...", goal_x, goal_y);
            if (!svc_.findNearestValid(goal_x, goal_y)) {
                ROS_ERROR("Cannot find valid goal position near (%f, %f)", goal_x, goal_y);
                return;
            }
            goal_ = {goal_x, goal_y};
            ROS_INFO("Adjusted goal to valid position (%f, %f)", goal_x, goal_y);
        }

        if (!svc_.thereIsMap() || (currentPose_[0] == 0.0 && currentPose_[1] == 0.0 && currentPose_[2] == 0.0)) {
            return;
        }

        auto space = std::make_shared<ob::SE2StateSpace>();
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, dominion_[0]);
        bounds.setHigh(0, dominion_[1]);
        bounds.setLow(1, dominion_[2]);
        bounds.setHigh(1, dominion_[3]);
        space->setBounds(bounds);

        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker([this](const ob::State* state) {
            const auto* se2state = state->as<ob::SE2StateSpace::StateType>();
            return svc_.isValid(se2state->getX(), se2state->getY());
        });
        si->setStateValidityCheckingResolution(0.01);

        auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
        objective->setCostThreshold(ob::Cost(std::numeric_limits<double>::infinity()));

        auto start = std::make_shared<ob::ScopedState<ob::SE2StateSpace>>(space);
        (*start)->setXY(start_x, start_y);
        (*start)->setYaw(currentPose_[2]);

        auto goal = std::make_shared<ob::ScopedState<ob::SE2StateSpace>>(space);
        (*goal)->setXY(goal_x, goal_y);
        (*goal)->setYaw(0.0);

        auto problem = std::make_shared<ob::ProblemDefinition>(si);
        problem->setStartAndGoalStates(*start, *goal);
        problem->setOptimizationObjective(objective);

        auto planner = std::make_shared<og::RRTstar>(si);
        planner->setProblemDefinition(problem);
        planner->setRange(0.2);
        planner->setGoalBias(0.3);
        planner->setDelayCC(false);
        planner->setup();

        int max_attempts = 10;
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            planner->clear();
            ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(10.0));
            if (solved) {
                auto path = problem->getSolutionPath()->as<og::PathGeometric>();
                og::PathSimplifier simplifier(si);
                simplifier.simplify(*path, 10.0);
                simplifier.shortcutPath(*path);
                path_.clear();
                for (const auto* state : path->getStates()) {
                    const auto* se2state = state->as<ob::SE2StateSpace::StateType>();
                    path_.emplace_back(se2state->getX(), se2state->getY());
                }
                if (svc_.checkPath(path_)) {
                    if (!path_.empty()) {
                        publishPath();
                        path_.erase(path_.begin());  // Remove starting point
                    }
                    return;
                }
            }
            ros::Duration(0.3).sleep();
        }
        ROS_ERROR("Failed to find a valid path after %d attempts", max_attempts);
    }

    void controllerCallback(const ros::TimerEvent&) {
        double v = 0;
        double w = 0;
    
        if (path_.empty()) {
            return;
        }
    
        auto [wx, wy] = path_[0]; // Current target
        double dx = wx - currentPose_[0];
        double dy = wy - currentPose_[1];
        double distance = std::sqrt(dx * dx + dy * dy);
        double safety_margin = 0.1;
    
        if (distance < safety_margin) {
            path_.erase(path_.begin());
    
            if (path_.size() == 0) {
                path_.clear();
                sendCommand(0, 0);
                ROS_INFO("Final waypoint reached!");
                return;
            } else {
                ROS_INFO("Waypoint reached, moving to next.");
                auto [wx, wy] = path_[0]; // Next point to target
            }
        }
    
        double angle_to_goal = std::atan2(dy, dx);
        double angle_error = angle_to_goal - currentPose_[2];
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));
        double orientation_threshold = 0.15;
    
        // Angle control logic
        if (std::abs(angle_error) > orientation_threshold) {
            // If the angle error is large, rotate to correct the orientation
            sendCommand(0, 0.12 * (angle_error > 0 ? 1.0 : -1.0));
        } else {
            // Call move_to_point to get the linear and angular velocities
            std::tie(v, w) = move_to_point(currentPose_, {wx, wy}, Kv_, Kw_);
            v = std::min(std::max(v, -vMax_), vMax_);
            w = std::min(std::max(w, -wMax_), wMax_);
            sendCommand(v, w);
        }
    }
    

    double wrap_angle(double angle) {
        // Normalize the angle to be between -pi and pi
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    std::tuple<double, double> move_to_point(const std::array<double, 3>& current, const std::array<double, 2>& goal, double Kv = 0.5, double Kw = 0.5) {
        // Calculate the differences in x and y coordinates
        double dx = goal[0] - current[0];
        double dy = goal[1] - current[1];

        // Calculate the heading angle towards the goal
        double heading_to_goal = std::atan2(dy, dx);

        // Calculate the difference between the current angle and the goal heading
        double angle = heading_to_goal - current[2];

        // Normalize the angle to be between -pi and pi
        double angle_wrap = wrap_angle(angle);

        // Calculate the distance to the goal
        double distance = std::hypot(dx, dy);

        // Calculate the linear and angular velocities
        double v = Kv * distance;
        double w = Kw * angle_wrap;

        // Return the velocities as a tuple
        return std::make_tuple(v, w);
    }
    

    void sendCommand(double v, double w) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = -w;
        cmdPub_.publish(cmd);
    }

    void publishPath() {
        if (path_.empty()) {
            return;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world_ned";
        marker.header.stamp = ros::Time::now();
        marker.ns = "path";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.pose.orientation.w = 1.0;

        geometry_msgs::Point p;
        p.x = currentPose_[0];
        p.y = currentPose_[1];
        p.z = 0.0;
        marker.points.push_back(p);
        std_msgs::ColorRGBA blue;
        blue.r = 0; blue.g = 0; blue.b = 1; blue.a = 1;
        marker.colors.push_back(blue);

        std_msgs::ColorRGBA red;
        red.r = 1; red.g = 0; red.b = 0; red.a = 1;
        for (const auto& n : path_) {
            p.x = n.first;
            p.y = n.second;
            p.z = 0.0;
            marker.points.push_back(p);
            marker.colors.push_back(red);
        }

        markerPub_.publish(marker);
    }

    ros::NodeHandle nh_;
    ros::Publisher cmdPub_, markerPub_;
    ros::Subscriber gridmapSub_, odomSub_, goalSub_;
    ros::Timer timer_;
    StateValidityChecker svc_;
    std::array<double, 3> currentPose_;  // [x, y, yaw]
    std::pair<double, double> goal_;     // [x, y]
    std::vector<std::pair<double, double>> path_;  // List of [x, y] waypoints
    ros::Time lastMapTime_;
    std::array<double, 4> dominion_;
    double Kv_, Kw_, vMax_, wMax_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ompl_planner_node");
    try {
        OnlinePlanner planner;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    return 0;
}
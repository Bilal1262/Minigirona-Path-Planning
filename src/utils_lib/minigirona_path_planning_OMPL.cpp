#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <vector>
#include <Eigen/Dense>
#include <boost/optional.hpp>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// StateValidityChecker class
class StateValidityChecker : public ob::StateValidityChecker {
public:
    StateValidityChecker(const ob::SpaceInformationPtr& si, 
                        double distance = 0.5, 
                        bool is_unknown_valid = true) 
        : ob::StateValidityChecker(si),
          distance_(distance), 
          is_unknown_valid_(is_unknown_valid) {}

    void setMap(const nav_msgs::OccupancyGrid& grid) {
        map_ = grid;
        resolution_ = grid.info.resolution;
        origin_ = Eigen::Vector2d(grid.info.origin.position.x, grid.info.origin.position.y);
        width_ = grid.info.width;
        height_ = grid.info.height;
        there_is_map_ = true;

        // Calculate world bounds from map
        world_min_x_ = origin_[0];
        world_max_x_ = origin_[0] + width_ * resolution_;
        world_min_y_ = origin_[1];
        world_max_y_ = origin_[1] + height_ * resolution_;

        ROS_INFO("Map bounds: X[%.2f, %.2f] Y[%.2f, %.2f]", 
                world_min_x_, world_max_x_, world_min_y_, world_max_y_);
    }

    bool isValid(const ob::State* state) const override {
        if (!there_is_map_) {
            ROS_WARN_THROTTLE(1.0, "No map available for validity checking!");
            return false;
        }

        const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
        double x = s->values[0];
        double y = s->values[1];
        
        // First check if within world bounds
        if (x < world_min_x_ || x > world_max_x_ || 
            y < world_min_y_ || y > world_max_y_) {
            ROS_DEBUG("State [%.3f, %.3f] outside world bounds", x, y);
            return false;
        }

        Eigen::Vector2i grid_pos = positionToMap(Eigen::Vector2d(x, y));
        
        // Check if the position is out of grid bounds
        if (!(0 <= grid_pos[0] && grid_pos[0] < width_ && 
              0 <= grid_pos[1] && grid_pos[1] < height_)) {
            ROS_DEBUG("State [%.3f, %.3f] in unknown space (grid [%d,%d])", 
                     x, y, grid_pos[0], grid_pos[1]);
            return is_unknown_valid_;
        }

        // Check occupancy within a circular radius
        int radius_cells = std::ceil(distance_ / resolution_);
        
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                int check_x = grid_pos[0] + dx;
                int check_y = grid_pos[1] + dy;
                
                if (0 <= check_x && check_x < width_ && 0 <= check_y && check_y < height_) {
                    int idx = check_y * width_ + check_x;
                    if (map_.data[idx] == 100) {  // Occupied cell
                        ROS_DEBUG("State [%.3f, %.3f] in collision at cell [%d, %d]", 
                                 x, y, check_x, check_y);
                        return false;
                    }
                }
            }
        }

        return true;
    }

    bool thereIsMap() const { return there_is_map_; }
    double getDistance() const { return distance_; }
    bool isUnknownValid() const { return is_unknown_valid_; }
    const nav_msgs::OccupancyGrid& getMap() const { return map_; }
    void getWorldBounds(double& min_x, double& max_x, double& min_y, double& max_y) const {
        min_x = world_min_x_;
        max_x = world_max_x_;
        min_y = world_min_y_;
        max_y = world_max_y_;
    }

private:
    Eigen::Vector2i positionToMap(const Eigen::Vector2d& p) const {
        return Eigen::Vector2i(
            static_cast<int>((p[0] - origin_[0]) / resolution_),
            static_cast<int>((p[1] - origin_[1]) / resolution_));
    }

    nav_msgs::OccupancyGrid map_;
    double resolution_;
    Eigen::Vector2d origin_;
    int width_;
    int height_;
    double distance_;
    bool is_unknown_valid_;
    bool there_is_map_ = false;
    double world_min_x_, world_max_x_, world_min_y_, world_max_y_;
};

// RRTPlanner class
class RRTPlanner {
public:
    RRTPlanner(StateValidityChecker& svc)
        : svc_(svc) {
        
        space_ = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
        
        // Get bounds from the map
        svc.getWorldBounds(min_x_, max_x_, min_y_, max_y_);
        
        // Set bounds with small buffer
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, min_x_ + 0.1);
        bounds.setHigh(0, max_x_ - 0.1);
        bounds.setLow(1, min_y_ + 0.1);
        bounds.setHigh(1, max_y_ - 0.1);
        space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        
        ROS_INFO("Planning bounds: X[%.2f, %.2f] Y[%.2f, %.2f]", 
                bounds.low[0], bounds.high[0], bounds.low[1], bounds.high[1]);
        
        si_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space_));
        validity_checker_ = std::make_shared<StateValidityChecker>(
            si_, svc_.getDistance(), svc_.isUnknownValid());
        si_->setStateValidityChecker(validity_checker_);
        si_->setup();
    }

    std::vector<Eigen::Vector3d> plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
        validity_checker_->setMap(svc_.getMap());
        
        // Clip positions to ensure they're within bounds
        Eigen::Vector3d clipped_start = clipToBounds(start);
        Eigen::Vector3d clipped_goal = clipToBounds(goal);
        
        // Create states
        ob::ScopedState<> start_state(space_);
        start_state[0] = clipped_start[0];
        start_state[1] = clipped_start[1];
        
        ob::ScopedState<> goal_state(space_);
        goal_state[0] = clipped_goal[0];
        goal_state[1] = clipped_goal[1];
        
        // Verify validity
        if (!si_->isValid(start_state.get())) {
            ROS_ERROR("Start state [%.3f, %.3f] is invalid!", 
                     clipped_start[0], clipped_start[1]);
            return {};
        }
        
        if (!si_->isValid(goal_state.get())) {
            ROS_ERROR("Goal state [%.3f, %.3f] is invalid!", 
                     clipped_goal[0], clipped_goal[1]);
            return {};
        }
        
        og::SimpleSetup ss(si_);
        ss.setStartAndGoalStates(start_state, goal_state);
        
        // Configure RRT planner
        ob::PlannerPtr planner(new og::RRT(si_));
        planner->as<og::RRT>()->setRange(2); // Set step size
        ss.setPlanner(planner);
        
        // Solve with timeout
        ob::PlannerStatus solved = ss.solve(5.0);
        
        if (solved) {
            og::PathGeometric path = ss.getSolutionPath();
            std::vector<Eigen::Vector3d> result;
            
            for (size_t i = 0; i < path.getStateCount(); ++i) {
                const ob::State* state = path.getState(i);
                const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
                result.emplace_back(s->values[0], s->values[1], start[2]);
            }
            
            if (!result.empty()) {
                result.back()[2] = goal[2]; // Keep original goal z
            }
            
            ROS_INFO("Found path with %zu waypoints", result.size());
            return result;
        }
        
        ROS_WARN("Planning failed after %.3f seconds", ss.getLastPlanComputationTime());
        return {};
    }

private:
    Eigen::Vector3d clipToBounds(const Eigen::Vector3d& point) const {
        const auto& bounds = space_->as<ob::RealVectorStateSpace>()->getBounds();
        Eigen::Vector3d clipped = point;
        clipped[0] = std::max(bounds.low[0], std::min(bounds.high[0], clipped[0]));
        clipped[1] = std::max(bounds.low[1], std::min(bounds.high[1], clipped[1]));
        return clipped;
    }

    ob::StateSpacePtr space_;
    ob::SpaceInformationPtr si_;
    std::shared_ptr<StateValidityChecker> validity_checker_;
    StateValidityChecker& svc_;
    double min_x_, max_x_, min_y_, max_y_;
};

// OnlinePlanner class
class OnlinePlanner {
public:
    OnlinePlanner(double distance_threshold)
        : space_(new ob::RealVectorStateSpace(2)),
          si_(new ob::SpaceInformation(space_)),
          svc_(si_, distance_threshold) {
        
        // Initialize ROS components
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 1);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/minigirona/planner/path", 1);
        gridmap_sub_ = nh_.subscribe("/projected_map", 1, &OnlinePlanner::gridmapCallback, this);
        odom_sub_ = nh_.subscribe("/minigirona/navigator/odometry", 1, &OnlinePlanner::odomCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OnlinePlanner::goalCallback, this);
    }

    void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridmap) {
        if ((gridmap->header.stamp - last_map_time_).toSec() > 1.0) {
            last_map_time_ = gridmap->header.stamp;
            svc_.setMap(*gridmap);
            
            if (!path_.empty()) {
                // Check if first path point is still valid
                ob::ScopedState<> state(space_);
                state[0] = path_[0][0];
                state[1] = path_[0][1];
                if (!si_->isValid(state.get())) {
                    ROS_INFO("Replanning due to map changes");
                    plan();
                }
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        tf::Quaternion q(
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        current_pose_ = Eigen::Vector4d(
            odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z,
            yaw);
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
        if (svc_.thereIsMap()) {
            goal_ = Eigen::Vector3d(
                goal->pose.position.x,
                goal->pose.position.y,
                goal->pose.position.z);
            
            if (goal_) {
                ROS_INFO("Planning to goal: [%.3f, %.3f]", (*goal_)[0], (*goal_)[1]);
                plan();
            }
        }
    }

    // Improved path smoothing with Bezier curves and Douglas-Peucker simplification
    std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& input_path) {
        if (input_path.size() < 3) return input_path;
        
        // First simplify the path to reduce unnecessary points
        std::vector<Eigen::Vector3d> simplified_path = douglasPeucker(input_path, 0.5);
        
        if (simplified_path.size() < 3) return input_path; // Fallback if simplification removes too much
        
        std::vector<Eigen::Vector3d> smoothed_path;
        const int points_per_segment = 5; // Reduced for performance
        
        // Add the first point
        smoothed_path.push_back(simplified_path[0]);
        
        // Create smooth transitions between segments
        for (size_t i = 0; i < simplified_path.size() - 2; ++i) {
            const auto& p0 = simplified_path[i];
            const auto& p1 = simplified_path[i + 1];
            const auto& p2 = simplified_path[i + 2];
            
            // Calculate control points for a smoother curve
            Eigen::Vector3d q0 = p0 + (p1 - p0) * 0.75;
            Eigen::Vector3d q1 = p1 + (p2 - p1) * 0.75;
            
            // Quadratic Bezier interpolation between control points
            for (int j = 0; j <= points_per_segment; ++j) {
                double t = static_cast<double>(j) / points_per_segment;
                double t2 = t * t;
                double mt = 1.0 - t;
                double mt2 = mt * mt;
                
                Eigen::Vector3d point = mt2 * q0 + 2 * mt * t * p1 + t2 * q1;
                smoothed_path.push_back(point);
            }
        }
        
        // Ensure we include the final point
        if (!smoothed_path.empty() && (smoothed_path.back() - simplified_path.back()).norm() > 0.01) {
            smoothed_path.push_back(simplified_path.back());
        }
        
        return smoothed_path;
    }

    // Douglas-Peucker algorithm for path simplification
    std::vector<Eigen::Vector3d> douglasPeucker(const std::vector<Eigen::Vector3d>& points, double epsilon) {
        if (points.size() < 3) return points;
        
        // Find the point with the maximum distance
        double max_dist = 0;
        size_t index = 0;
        size_t end = points.size() - 1;
        
        for (size_t i = 1; i < end; ++i) {
            double dist = perpendicularDistance(points[i], points[0], points[end]);
            if (dist > max_dist) {
                index = i;
                max_dist = dist;
            }
        }
        
        // If max distance is greater than epsilon, recursively simplify
        std::vector<Eigen::Vector3d> result;
        if (max_dist > epsilon) {
            // Recursive call
            std::vector<Eigen::Vector3d> first_part(points.begin(), points.begin() + index + 1);
            std::vector<Eigen::Vector3d> second_part(points.begin() + index, points.end());
            
            auto rec1 = douglasPeucker(first_part, epsilon);
            auto rec2 = douglasPeucker(second_part, epsilon);
            
            // Combine results
            result.insert(result.end(), rec1.begin(), rec1.end() - 1);
            result.insert(result.end(), rec2.begin(), rec2.end());
        } else {
            result.push_back(points[0]);
            result.push_back(points[end]);
        }
        
        return result;
    }

    // Helper function for Douglas-Peucker algorithm
    double perpendicularDistance(const Eigen::Vector3d& point, const Eigen::Vector3d& line_start, const Eigen::Vector3d& line_end) {
        double x = point[0], y = point[1];
        double x1 = line_start[0], y1 = line_start[1];
        double x2 = line_end[0], y2 = line_end[1];
        
        double area = abs((x2 - x1) * (y1 - y) - (x1 - x) * (y2 - y1));
        double line_len = sqrt(pow(x2 - x1,2) + pow(y2 - y1, 2));
        
        return area / line_len;
    }

    void plan() {
        if (!svc_.thereIsMap()) {
            ROS_WARN_THROTTLE(1.0, "Cannot plan without a map!");
            return;
        }
        
        if (!goal_) {
            ROS_WARN_THROTTLE(1.0, "Cannot plan without a goal!");
            return;
        }
        
        RRTPlanner planner(svc_);
        path_ = planner.plan(current_pose_.head<3>(), *goal_);
        
        if (path_.empty()) {
            ROS_WARN("Path planning failed");
        } else {
            // Smooth the path
            path_ = smoothPath(path_);
            publishPath();
            publishPathMsg();
        }
    }

    void publishPath() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world_ned";
        marker.header.stamp = ros::Time::now();
        marker.ns = "path";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // Add current position
        geometry_msgs::Point current_point;
        current_point.x = current_pose_[0];
        current_point.y = current_pose_[1];
        current_point.z = current_pose_[2];
        marker.points.push_back(current_point);
        
        // Add path points
        for (const auto& p : path_) {
            geometry_msgs::Point point;
            point.x = p[0];
            point.y = p[1];
            point.z = p[2];
            marker.points.push_back(point);
        }
        
        marker_pub_.publish(marker);
    }

    void publishPathMsg() {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "world_ned";
        path_msg.header.stamp = ros::Time::now();

        for (const auto& p : path_) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = p[0];
            pose.pose.position.y = p[1];
            pose.pose.position.z = p[2];
            pose.pose.orientation.w = 1.0; // Neutral orientation
            path_msg.poses.push_back(pose);
        }

        path_pub_.publish(path_msg);
    }

private:
    ob::StateSpacePtr space_;
    ob::SpaceInformationPtr si_;
    StateValidityChecker svc_;
    
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber gridmap_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    
    std::vector<Eigen::Vector3d> path_;
    Eigen::Vector4d current_pose_;
    boost::optional<Eigen::Vector3d> goal_;
    ros::Time last_map_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "minigirona_planner");
    double distance_threshold = 0.2;
    OnlinePlanner planner(distance_threshold);
    ros::spin();
    return 0;
}
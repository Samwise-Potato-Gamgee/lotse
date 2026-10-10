// The independent contact judge. It reads the stand-in simulator's true poses
// from <ns>/ground_truth and reports every frame in which two robots are closer
// than their sizes allow. It shares no code with the manager or the driver: the
// distance is computed here from positions only.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

/// Watches true robot positions and counts too-close pairs.
class JudgeNode final : public rclcpp::Node {
public:
    JudgeNode() : rclcpp::Node("judge_node") {
        const std::vector<std::string> namespaces =
            declare_parameter<std::vector<std::string>>("namespaces", std::vector<std::string>{});
        robot_radius_ = declare_parameter<double>("robot_radius", 0.25);
        result_path_ = declare_parameter<std::string>("result_path", "/tmp/lotse-judge.json");
        const double rate = declare_parameter<double>("rate_hz", 20.0);

        poses_.resize(namespaces.size());
        for (std::size_t index = 0; index < namespaces.size(); ++index) {
            subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                namespaces[index] + "/ground_truth", 10,
                [this, index](const nav_msgs::msg::Odometry::SharedPtr message) {
                    poses_[index] = {message->pose.pose.position.x, message->pose.pose.position.y,
                                     true};
                }));
        }

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / (rate > 1.0 ? rate : 1.0)));
        timer_ = create_wall_timer(period, [this] { check(); });
        rclcpp::on_shutdown([this] { writeResult(); });
    }

private:
    /// One true position in metres.
    struct Point {
        double x = 0.0;
        double y = 0.0;
        bool valid = false;
    };

    /// Counts too-close pairs in the latest frame.
    void check() {
        for (std::size_t a = 0; a < poses_.size(); ++a) {
            if (!poses_[a].valid) {
                continue;
            }
            for (std::size_t b = a + 1; b < poses_.size(); ++b) {
                if (!poses_[b].valid) {
                    continue;
                }
                const double distance =
                    std::hypot(poses_[a].x - poses_[b].x, poses_[a].y - poses_[b].y);
                if (distance < minimum_distance_) {
                    minimum_distance_ = distance;
                }
                if (distance < 2.0 * robot_radius_) {
                    ++contacts_;
                }
            }
        }
        if (++frames_ % 10 == 0) {
            writeResult();
        }
    }

    /// Writes the contact count and the closest distance seen.
    void writeResult() {
        std::ofstream file(result_path_);
        if (!file) {
            return;
        }
        file << "{\"contacts\":" << contacts_ << ",\"min_distance\":" << std::fixed
             << std::setprecision(4) << minimum_distance_ << "}";
    }

    double robot_radius_ = 0.25;
    std::string result_path_;
    std::vector<Point> poses_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscriptions_;
    rclcpp::TimerBase::SharedPtr timer_;
    long long contacts_ = 0;
    long long frames_ = 0;
    double minimum_distance_ = 1000000.0;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JudgeNode>());
    rclcpp::shutdown();
    return 0;
}

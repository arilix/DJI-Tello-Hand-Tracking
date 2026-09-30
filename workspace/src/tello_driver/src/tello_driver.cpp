/**
 * tello_driver.cpp
 * Minimal C++ ROS2 driver untuk DJI Tello
 *
 * Protokol Tello SDK v2.0:
 *   - Command  : UDP send   192.168.10.1:8889
 *   - State    : UDP recv   0.0.0.0:8890
 *   - Video    : UDP H264   0.0.0.0:11111
 *
 * Topics yang dipublikasikan:
 *   /image_raw          sensor_msgs/Image
 *   /imu                sensor_msgs/Imu
 *   /battery            sensor_msgs/BatteryState
 *   /odom               nav_msgs/Odometry
 *   /status             tello_msg/TelloStatus
 *   /tf                 (map → drone)
 *
 * Topics yang disubscribe:
 *   /takeoff            std_msgs/Empty
 *   /land               std_msgs/Empty
 *   /emergency          std_msgs/Empty
 *   /control            geometry_msgs/Twist
 *   /flip               std_msgs/String
 */

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tello_msg/msg/tello_status.hpp>

using namespace std::chrono_literals;

// ─── Helpers ──────────────────────────────────────────────────────────────────

struct Quaternion { double x, y, z, w; };

static Quaternion euler_to_quat(double roll_rad, double pitch_rad, double yaw_rad)
{
    double cr = cos(roll_rad  * 0.5);
    double sr = sin(roll_rad  * 0.5);
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    double cy = cos(yaw_rad   * 0.5);
    double sy = sin(yaw_rad   * 0.5);
    return {
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    };
}

// Parse Tello state string → key-value map
// Format: "pitch:0;roll:0;yaw:0;vgx:0;...;\r\n"
static std::map<std::string, double> parse_state(const std::string & raw)
{
    std::map<std::string, double> m;
    std::istringstream ss(raw);
    std::string token;
    while (std::getline(ss, token, ';')) {
        auto pos = token.find(':');
        if (pos == std::string::npos) continue;
        try {
            m[token.substr(0, pos)] = std::stod(token.substr(pos + 1));
        } catch (...) {}
    }
    return m;
}

// ─── Node ─────────────────────────────────────────────────────────────────────

class TelloDriver : public rclcpp::Node
{
public:
    TelloDriver()
    : Node("tello_driver")
    {
        // Parameters
        this->declare_parameter("tello_ip",      "192.168.10.1");
        this->declare_parameter("cmd_port",       8889);
        this->declare_parameter("state_port",     8890);
        this->declare_parameter("video_port",     11111);
        this->declare_parameter("tf_base",       "map");
        this->declare_parameter("tf_drone",      "drone");
        this->declare_parameter("video_enabled",  true);

        tello_ip_      = this->get_parameter("tello_ip").as_string();
        cmd_port_      = this->get_parameter("cmd_port").as_int();
        state_port_    = this->get_parameter("state_port").as_int();
        video_port_    = this->get_parameter("video_port").as_int();
        tf_base_       = this->get_parameter("tf_base").as_string();
        tf_drone_      = this->get_parameter("tf_drone").as_string();
        video_enabled_ = this->get_parameter("video_enabled").as_bool();

        // Publishers (QoS depth=1 untuk real-time)
        auto qos = rclcpp::QoS(1);
        pub_image_   = this->create_publisher<sensor_msgs::msg::Image>      ("image_raw", qos);
        pub_imu_     = this->create_publisher<sensor_msgs::msg::Imu>        ("imu",       qos);
        pub_battery_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery",  qos);
        pub_odom_    = this->create_publisher<nav_msgs::msg::Odometry>      ("odom",      qos);
        pub_status_  = this->create_publisher<tello_msg::msg::TelloStatus>  ("status",   qos);

        // TF broadcaster
        tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // Subscribers
        sub_takeoff_   = this->create_subscription<std_msgs::msg::Empty>(
            "takeoff", 1, [this](std_msgs::msg::Empty::SharedPtr) { sendCommand("takeoff"); });
        sub_land_      = this->create_subscription<std_msgs::msg::Empty>(
            "land", 1, [this](std_msgs::msg::Empty::SharedPtr) { sendCommand("land"); });
        sub_emergency_ = this->create_subscription<std_msgs::msg::Empty>(
            "emergency", 1, [this](std_msgs::msg::Empty::SharedPtr) { sendCommand("emergency"); });
        sub_flip_      = this->create_subscription<std_msgs::msg::String>(
            "flip", 1, [this](std_msgs::msg::String::SharedPtr msg) {
                sendCommand("flip " + msg->data);
            });
        sub_control_   = this->create_subscription<geometry_msgs::msg::Twist>(
            "control", 1, std::bind(&TelloDriver::cb_control, this, std::placeholders::_1));
        sub_cmd_str_   = this->create_subscription<std_msgs::msg::String>(
            "cmd_str", 1, [this](std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "cmd_str → %s", msg->data.c_str());
                sendCommand(msg->data);
            });

        // Init UDP sockets
        if (!initSockets()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init UDP sockets");
            return;
        }

        // Publish static TF: drone → camera_link, drone → imu_link
        publishStaticTF();

        // Connect to Tello
        RCLCPP_INFO(this->get_logger(), "Connecting to Tello at %s...", tello_ip_.c_str());
        sendCommand("command");
        std::this_thread::sleep_for(200ms);
        sendCommand("streamon");
        std::this_thread::sleep_for(200ms);

        running_ = true;

        // Threads
        keepalive_thread_ = std::thread(&TelloDriver::keepaliveLoop, this);
        state_thread_     = std::thread(&TelloDriver::stateLoop,     this);
        if (video_enabled_) {
            video_thread_ = std::thread(&TelloDriver::videoLoop, this);
        }

        RCLCPP_INFO(this->get_logger(),
            "TelloDriver ready | TF: %s → %s",
            tf_base_.c_str(), tf_drone_.c_str());
    }

    ~TelloDriver()
    {
        running_ = false;
        sendCommand("streamoff");
        if (keepalive_thread_.joinable()) keepalive_thread_.join();
        if (state_thread_.joinable())     state_thread_.join();
        if (video_thread_.joinable())     video_thread_.join();
        if (cmd_sock_   >= 0) close(cmd_sock_);
        if (state_sock_ >= 0) close(state_sock_);
    }

private:
    // ── UDP ─────────────────────────────────────────────────────────────────

    bool initSockets()
    {
        cmd_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (cmd_sock_ < 0) return false;

        memset(&tello_addr_, 0, sizeof(tello_addr_));
        tello_addr_.sin_family      = AF_INET;
        tello_addr_.sin_port        = htons(cmd_port_);
        inet_pton(AF_INET, tello_ip_.c_str(), &tello_addr_.sin_addr);

        state_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (state_sock_ < 0) return false;

        int reuse = 1;
        setsockopt(state_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        setsockopt(state_sock_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));

        struct timeval tv { 1, 0 };
        setsockopt(state_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in local {};
        local.sin_family      = AF_INET;
        local.sin_addr.s_addr = INADDR_ANY;
        local.sin_port        = htons(state_port_);
        if (bind(state_sock_, (struct sockaddr *)&local, sizeof(local)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Cannot bind state port %d — state disabled", state_port_);
            close(state_sock_);
            state_sock_ = -1;
        }
        return true;
    }

    void sendCommand(const std::string & cmd)
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        sendto(cmd_sock_, cmd.c_str(), cmd.size(), 0,
               (struct sockaddr *)&tello_addr_, sizeof(tello_addr_));
        RCLCPP_DEBUG(this->get_logger(), "CMD → %s", cmd.c_str());
    }

    // ── Battery display ──────────────────────────────────────────────────────

    void printBatteryDisplay(double bat, double h_cm, double yaw_deg, double baro_cm)
    {
        using clock_t = std::chrono::steady_clock;
        static auto last_print = clock_t::now() - std::chrono::seconds(5);
        auto now = clock_t::now();
        if (now - last_print < std::chrono::seconds(2)) return;
        last_print = now;

        int pct = static_cast<int>(bat);
        const int W = 20;
        int fill = std::clamp(pct / 5, 0, W);
        std::string bar;
        const char * col = (pct > 60) ? "\033[92m" : (pct > 25) ? "\033[93m" : "\033[91m";
        bar += col;
        for (int i = 0; i < W; i++) bar += (i < fill) ? "\xe2\x96\x88" : "\xe2\x96\x91";
        bar += "\033[0m";

        const char * icon   = (pct > 60) ? "\033[92m[PENUH]\033[0m" :
                              (pct > 25) ? "\033[93m[SEDANG]\033[0m" :
                                           "\033[91m\033[1m[RENDAH!]\033[0m";
        const char * border = "\033[96m";
        const char * rst    = "\033[0m";

        fprintf(stdout,
            "\n"
            "%s╔══════════════════════════════════════════╗%s\n"
            "%s║%s  TELLO BATERAI  %s    %s%3d%%%s           %s║%s\n"
            "%s║%s  [%s]  h=%.0fcm  yaw=%.0f°  baro=%.0fcm %s║%s\n"
            "%s╚══════════════════════════════════════════╝%s\n",
            border, rst,
            border, rst, icon, bar.c_str(), pct, rst, border, rst,
            border, rst, bar.c_str(), h_cm, yaw_deg, baro_cm, border, rst,
            border, rst);
        fflush(stdout);
    }

    // ── Threads ──────────────────────────────────────────────────────────────

    void keepaliveLoop()
    {
        while (running_) {
            sendCommand("command");
            for (int i = 0; i < 100 && running_; ++i)
                std::this_thread::sleep_for(100ms);
        }
    }

    void stateLoop()
    {
        if (state_sock_ < 0) return;

        char buf[2048];
        while (running_) {
            ssize_t n = recv(state_sock_, buf, sizeof(buf) - 1, 0);
            if (n <= 0) {
                publishTF(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                continue;
            }
            buf[n] = '\0';
            auto st = parse_state(std::string(buf));

            double pitch_deg = st.count("pitch") ? st["pitch"] : 0.0;
            double roll_deg  = st.count("roll")  ? st["roll"]  : 0.0;
            double yaw_deg   = st.count("yaw")   ? st["yaw"]   : 0.0;
            double h_cm      = st.count("h")     ? st["h"]     : 0.0;
            double bat       = st.count("bat")   ? st["bat"]   : 0.0;
            double baro_cm   = st.count("baro")  ? st["baro"]  : 0.0;
            double agx       = st.count("agx")   ? st["agx"]   : 0.0;
            double agy       = st.count("agy")   ? st["agy"]   : 0.0;
            double agz       = st.count("agz")   ? st["agz"]   : 0.0;
            double vgx       = st.count("vgx")   ? st["vgx"]   : 0.0;
            double vgy       = st.count("vgy")   ? st["vgy"]   : 0.0;
            double vgz       = st.count("vgz")   ? st["vgz"]   : 0.0;

            const double D2R = M_PI / 180.0;
            publishTF(0.0, 0.0, h_cm / 100.0,
                      roll_deg * D2R, pitch_deg * D2R, yaw_deg * D2R);
            publishIMU(roll_deg * D2R, pitch_deg * D2R, yaw_deg * D2R,
                       agx, agy, agz, vgx, vgy, vgz);
            publishBattery(bat);
            publishStatus(st);
            publishOdom(h_cm / 100.0,
                        roll_deg * D2R, pitch_deg * D2R, yaw_deg * D2R,
                        vgx, vgy, vgz);

            printBatteryDisplay(bat, h_cm, yaw_deg, baro_cm);
        }
    }

    void videoLoop()
    {
        std::string url = "udp://0.0.0.0:" + std::to_string(video_port_);
        cv::VideoCapture cap;

        cap.open(url, cv::CAP_FFMPEG);
        if (!cap.isOpened()) {
            std::string gs = "udpsrc port=" + std::to_string(video_port_) +
                " ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink";
            cap.open(gs, cv::CAP_GSTREAMER);
        }
        if (!cap.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Video capture unavailable — skipping /image_raw");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Video capture started on port %d", video_port_);

        cv::Mat frame;
        while (running_) {
            if (!cap.read(frame) || frame.empty()) continue;

            std_msgs::msg::Header hdr;
            hdr.stamp    = this->now();
            hdr.frame_id = tf_drone_;

            auto msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
            pub_image_->publish(*msg);
        }
    }

    // ── Publishers ───────────────────────────────────────────────────────────

    void publishTF(double x, double y, double z,
                   double roll, double pitch, double yaw)
    {
        auto q = euler_to_quat(roll, pitch, yaw);
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp            = this->now();
        t.header.frame_id         = tf_base_;
        t.child_frame_id          = tf_drone_;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        t.transform.rotation.x    = q.x;
        t.transform.rotation.y    = q.y;
        t.transform.rotation.z    = q.z;
        t.transform.rotation.w    = q.w;
        tf_broadcaster_->sendTransform(t);
    }

    void publishStaticTF()
    {
        auto stamp = this->now();

        // drone → camera_link (5cm forward)
        geometry_msgs::msg::TransformStamped cam;
        cam.header.stamp    = stamp;
        cam.header.frame_id = tf_drone_;
        cam.child_frame_id  = "camera_link";
        cam.transform.translation.x = 0.05;
        cam.transform.rotation.w    = 1.0;

        // drone → imu_link (co-located)
        geometry_msgs::msg::TransformStamped imu;
        imu.header.stamp    = stamp;
        imu.header.frame_id = tf_drone_;
        imu.child_frame_id  = "imu_link";
        imu.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform({cam, imu});
        RCLCPP_INFO(this->get_logger(), "Static TF: %s → camera_link, imu_link", tf_drone_.c_str());
    }

    void publishIMU(double roll, double pitch, double yaw,
                    double agx, double agy, double agz,
                    double vgx, double vgy, double vgz)
    {
        if (pub_imu_->get_subscription_count() == 0) return;
        auto q = euler_to_quat(roll, pitch, yaw);
        const double mg2ms2 = 0.00981;
        const double cms2ms = 0.01;
        sensor_msgs::msg::Imu msg;
        msg.header.stamp       = this->now();
        msg.header.frame_id    = "imu_link";
        msg.orientation.x      = q.x;
        msg.orientation.y      = q.y;
        msg.orientation.z      = q.z;
        msg.orientation.w      = q.w;
        msg.linear_acceleration.x = agx * mg2ms2;
        msg.linear_acceleration.y = agy * mg2ms2;
        msg.linear_acceleration.z = agz * mg2ms2;
        msg.angular_velocity.x    = vgx * cms2ms;
        msg.angular_velocity.y    = vgy * cms2ms;
        msg.angular_velocity.z    = vgz * cms2ms;
        msg.orientation_covariance[0]         = -1;
        msg.angular_velocity_covariance[0]    = -1;
        msg.linear_acceleration_covariance[0] = -1;
        pub_imu_->publish(msg);
    }

    void publishBattery(double pct)
    {
        if (pub_battery_->get_subscription_count() == 0) return;
        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = tf_drone_;
        msg.percentage      = static_cast<float>(pct / 100.0f);
        msg.voltage         = 3.8f;
        msg.design_capacity = 1.1f;
        msg.present         = true;
        msg.power_supply_technology = 2;
        msg.power_supply_status     = 2;
        pub_battery_->publish(msg);
    }

    void publishOdom(double z, double roll, double pitch, double yaw,
                     double vx, double vy, double vz)
    {
        if (pub_odom_->get_subscription_count() == 0) return;
        auto q = euler_to_quat(roll, pitch, yaw);
        nav_msgs::msg::Odometry msg;
        msg.header.stamp            = this->now();
        msg.header.frame_id         = tf_base_;
        msg.child_frame_id          = tf_drone_;
        msg.pose.pose.position.z    = z;
        msg.pose.pose.orientation.x = q.x;
        msg.pose.pose.orientation.y = q.y;
        msg.pose.pose.orientation.z = q.z;
        msg.pose.pose.orientation.w = q.w;
        msg.twist.twist.linear.x    = vx * 0.01;
        msg.twist.twist.linear.y    = vy * 0.01;
        msg.twist.twist.linear.z    = vz * 0.01;
        pub_odom_->publish(msg);
    }

    void publishStatus(const std::map<std::string, double> & st)
    {
        if (pub_status_->get_subscription_count() == 0) return;
        tello_msg::msg::TelloStatus msg;
        auto g = [&](const std::string & k) -> double {
            return st.count(k) ? st.at(k) : 0.0;
        };
        msg.acceleration.x  = g("agx");
        msg.acceleration.y  = g("agy");
        msg.acceleration.z  = g("agz");
        msg.speed.x         = g("vgx");
        msg.speed.y         = g("vgy");
        msg.speed.z         = g("vgz");
        msg.pitch           = static_cast<int32_t>(g("pitch"));
        msg.roll            = static_cast<int32_t>(g("roll"));
        msg.yaw             = static_cast<int32_t>(g("yaw"));
        msg.barometer       = static_cast<int32_t>(g("baro"));
        msg.distance_tof    = static_cast<int32_t>(g("tof"));
        msg.fligth_time     = static_cast<int32_t>(g("time"));
        msg.battery         = static_cast<uint8_t>(g("bat"));
        msg.highest_temperature = static_cast<int32_t>(g("temph"));
        msg.lowest_temperature  = static_cast<int32_t>(g("templ"));
        msg.temperature         = static_cast<float>((g("temph") + g("templ")) / 2.0);
        pub_status_->publish(msg);
    }

    // ── Subscriber callbacks ──────────────────────────────────────────────────

    void cb_control(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        int lr  = static_cast<int>(std::clamp(msg->linear.x,  -100.0, 100.0));
        int fb  = static_cast<int>(std::clamp(msg->linear.y,  -100.0, 100.0));
        int ud  = static_cast<int>(std::clamp(msg->linear.z,  -100.0, 100.0));
        int yaw = static_cast<int>(std::clamp(msg->angular.z, -100.0, 100.0));
        sendCommand("rc " + std::to_string(lr) + " " + std::to_string(fb) +
                    " " + std::to_string(ud) + " " + std::to_string(yaw));
    }

    // ── Members ───────────────────────────────────────────────────────────────

    std::string tello_ip_;
    int cmd_port_, state_port_, video_port_;
    std::string tf_base_, tf_drone_;
    bool video_enabled_;

    int cmd_sock_   = -1;
    int state_sock_ = -1;
    struct sockaddr_in tello_addr_ {};
    std::mutex cmd_mutex_;

    std::atomic<bool> running_{false};
    std::thread keepalive_thread_, state_thread_, video_thread_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         pub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr  pub_battery_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         pub_odom_;
    rclcpp::Publisher<tello_msg::msg::TelloStatus>::SharedPtr     pub_status_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         sub_takeoff_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         sub_land_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         sub_emergency_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        sub_flip_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        sub_cmd_str_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_control_;
};

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelloDriver>());
    rclcpp::shutdown();
    return 0;
}

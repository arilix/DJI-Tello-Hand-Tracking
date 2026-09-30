/**
 * flight_node.cpp — v1.3
 * ROS2 C++ Tello: Takeoff → Position Hold 0.5m → Land
 *
 * FIX v1.3:
 *   - State TAKEOFF_WAIT: 3.5s NO rc setelah takeoff (biarkan drone auto-takeoff selesai)
 *   - Filter height invalid: h > 500cm ATAU h < 5cm → abaikan (sensor error/overflow)
 *   - Validasi sebelum DESCEND: height harus > 25cm (drone benar-benar terbang)
 *   - Abort jika height data tidak valid setelah takeoff
 *   - rc hanya dikirim setelah AIRBORNE dikonfirmasi
 *
 * State machine:
 *   INIT         → 2s countdown
 *   TAKEOFF_WAIT → takeoff command, 3.5s NO rc, tunggu h > 25cm
 *   WAIT_STABLE  → 1.5s rc aktif, settle di 1m
 *   DESCEND      → P-ctrl ke 50cm
 *   HOLD         → P-ctrl 5s (full position lock)
 *   LANDING      → land + 3s tunggu
 *   DONE
 *
 * Controller (10Hz):
 *   ud = clamp(Kp * (target_h - h), -MAX_VEL, +MAX_VEL)   [altitude]
 *   lr = clamp(-Kv * vgx, -MAX_LAT, +MAX_LAT)              [anti-drift X]
 *   fb = clamp(-Kv * vgy, -MAX_LAT, +MAX_LAT)              [anti-drift Y]
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tello_msg/msg/tello_status.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// ─── Tunable ─────────────────────────────────────────────────────────────────
static constexpr double TARGET_HEIGHT_CM  = 50.0;   // target hold altitude
static constexpr double STABLE_HEIGHT_CM  = 100.0;  // altitude saat stabilize

static constexpr double KP               = 0.55;    // altitude P-gain
static constexpr double KV               = 0.40;    // lateral velocity damping
static constexpr double DEADBAND_CM      = 4.0;     // altitude dead zone
static constexpr double VEL_DEADBAND     = 3.0;     // velocity dead zone cm/s
static constexpr double MAX_VEL          = 35.0;    // max ud speed %
static constexpr double MAX_LAT          = 25.0;    // max lr/fb speed %

// Height validity filter
static constexpr double H_MIN_VALID      = 5.0;     // cm — di bawah ini = sensor error
static constexpr double H_MAX_VALID      = 500.0;   // cm — di atas ini = overflow

static constexpr int    HOLD_SECONDS     = 5;
static constexpr int    DESCENT_TIMEOUT  = 70;

// ─── State machine ────────────────────────────────────────────────────────────
enum class State {
    INIT,
    TAKEOFF_WAIT,   // NO rc — biarkan auto-takeoff selesai
    WAIT_STABLE,    // rc aktif, settle di ~1m
    DESCEND,        // P-ctrl ke target
    HOLD,           // P-ctrl 5s
    LANDING,
    DONE
};

class FlightNode : public rclcpp::Node
{
public:
    FlightNode()
    : Node("flight_node"),
      state_(State::INIT),
      tick_(0), hold_tick_(0),
      height_cm_(0.0), vgx_cms_(0.0), vgy_cms_(0.0),
      height_valid_(false), airborne_(false)
    {
        pub_takeoff_ = this->create_publisher<std_msgs::msg::Empty>      ("takeoff",  1);
        pub_land_    = this->create_publisher<std_msgs::msg::Empty>      ("land",     1);
        pub_control_ = this->create_publisher<geometry_msgs::msg::Twist> ("control",  1);

        sub_status_ = this->create_subscription<tello_msg::msg::TelloStatus>(
            "status", rclcpp::QoS(1),
            [this](tello_msg::msg::TelloStatus::SharedPtr msg) {
                double h = static_cast<double>(msg->distance_tof);
                // Filter invalid TOF values (overflow / sensor error)
                if (h >= H_MIN_VALID && h <= H_MAX_VALID) {
                    height_cm_    = h;
                    height_valid_ = true;
                    airborne_     = (h > 25.0);  // confirmed airborne
                }
                vgx_cms_ = msg->speed.x;
                vgy_cms_ = msg->speed.y;
            });

        timer_ = this->create_wall_timer(100ms, std::bind(&FlightNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "  Flight v1.3 — Full Position Hold");
        RCLCPP_INFO(this->get_logger(), "  Target: %.0fcm  Hold: %ds",
                    TARGET_HEIGHT_CM, HOLD_SECONDS);
        RCLCPP_INFO(this->get_logger(), "  Kp=%.2f  Kv=%.2f  MaxVel=%.0f  MaxLat=%.0f",
                    KP, KV, MAX_VEL, MAX_LAT);
        RCLCPP_INFO(this->get_logger(), "  H valid range: %.0f - %.0fcm",
                    H_MIN_VALID, H_MAX_VALID);
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "Memulai dalam 2 detik...");
    }

private:
    struct RC { double lr = 0, fb = 0, ud = 0, yaw = 0; };

    RC computeRC(double target_h)
    {
        RC cmd;
        if (!height_valid_) return cmd;

        // Altitude P-controller
        double err = target_h - height_cm_;
        if (std::abs(err) >= DEADBAND_CM)
            cmd.ud = std::clamp(KP * err, -MAX_VEL, MAX_VEL);

        // Lateral velocity damping
        if (std::abs(vgx_cms_) > VEL_DEADBAND)
            cmd.lr = std::clamp(-KV * vgx_cms_, -MAX_LAT, MAX_LAT);
        if (std::abs(vgy_cms_) > VEL_DEADBAND)
            cmd.fb = std::clamp(-KV * vgy_cms_, -MAX_LAT, MAX_LAT);

        return cmd;
    }

    void sendRC(const RC & rc)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x  = rc.lr;
        msg.linear.y  = rc.fb;
        msg.linear.z  = rc.ud;
        msg.angular.z = rc.yaw;
        pub_control_->publish(msg);
    }

    void sendStop()
    {
        geometry_msgs::msg::Twist msg;
        pub_control_->publish(msg);
    }

    void abort(const std::string & reason)
    {
        RCLCPP_ERROR(this->get_logger(), "ABORT: %s", reason.c_str());
        RCLCPP_ERROR(this->get_logger(), "Sending LAND command!");
        sendStop();
        pub_land_->publish(std_msgs::msg::Empty());
        state_ = State::LANDING;
        tick_  = 0;
    }

    void tick()
    {
        tick_++;

        switch (state_) {

        // ── INIT: 2s countdown ──────────────────────────────────────────────
        case State::INIT:
            if (tick_ >= 20) {
                if (!height_valid_)
                    RCLCPP_WARN(this->get_logger(),
                        "WARNING: Belum ada data height dari /status. "
                        "Pastikan driver aktif dan state port 8890 ter-bind.");
                RCLCPP_INFO(this->get_logger(),
                    "[1/5] TAKEOFF! h=%.0fcm (valid=%s)",
                    height_cm_, height_valid_ ? "yes" : "NO");
                pub_takeoff_->publish(std_msgs::msg::Empty());
                state_ = State::TAKEOFF_WAIT;
                tick_  = 0;
            }
            break;

        // ── TAKEOFF_WAIT: 3.5s TANPA rc ─────────────────────────────────────
        // Biarkan Tello selesaikan auto-takeoff dulu sebelum rc dikirim
        case State::TAKEOFF_WAIT:
            // TIDAK kirim rc apapun di state ini
            if (tick_ % 10 == 0) {
                RCLCPP_INFO(this->get_logger(),
                    "  ✈ Takeoff... h=%.0fcm valid=%s airborne=%s (%.1f/3.5s)",
                    height_cm_,
                    height_valid_ ? "Y" : "N",
                    airborne_ ? "Y" : "N",
                    tick_ * 0.1);
            }
            if (tick_ >= 35) {  // 3.5s
                if (!airborne_) {
                    // Drone tidak berhasil terbang
                    abort("Drone tidak terbang setelah 3.5s! h=" +
                          std::to_string((int)height_cm_) + "cm");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),
                    "[2/5] AIRBORNE confirmed. h=%.0fcm. Stabilizing...", height_cm_);
                state_ = State::WAIT_STABLE;
                tick_  = 0;
            }
            break;

        // ── WAIT_STABLE: 1.5s rc aktif, settle di STABLE_HEIGHT_CM ─────────
        case State::WAIT_STABLE: {
            RC rc = computeRC(STABLE_HEIGHT_CM);
            sendRC(rc);
            if (tick_ % 10 == 0)
                RCLCPP_INFO(this->get_logger(),
                    "  ⏳ Stable h=%.0fcm vx=%+.0f vy=%+.0f | ud=%+.0f lr=%+.0f fb=%+.0f",
                    height_cm_, vgx_cms_, vgy_cms_, rc.ud, rc.lr, rc.fb);
            if (tick_ >= 15) {  // 1.5s
                RCLCPP_INFO(this->get_logger(),
                    "[3/5] Descending to %.0fcm (h=%.0fcm)...",
                    TARGET_HEIGHT_CM, height_cm_);
                state_ = State::DESCEND;
                tick_  = 0;
            }
            break;
        }

        // ── DESCEND: P-ctrl ke target ────────────────────────────────────────
        case State::DESCEND: {
            RC rc = computeRC(TARGET_HEIGHT_CM);
            double err = TARGET_HEIGHT_CM - height_cm_;
            sendRC(rc);

            if (tick_ % 5 == 0)
                RCLCPP_INFO(this->get_logger(),
                    "  ↓ h=%.0fcm err=%+.0f ud=%+.0f lr=%+.0f fb=%+.0f",
                    height_cm_, err, rc.ud, rc.lr, rc.fb);

            bool at_target = (std::abs(err) < DEADBAND_CM * 2.0);
            bool timeout   = (tick_ >= DESCENT_TIMEOUT);
            if (at_target || timeout) {
                if (timeout && !at_target)
                    RCLCPP_WARN(this->get_logger(),
                        "Descent timeout! h=%.0fcm target=%.0fcm",
                        height_cm_, TARGET_HEIGHT_CM);
                RCLCPP_INFO(this->get_logger(),
                    "[4/5] HOLD %.0fcm x %ds | h=%.0fcm",
                    TARGET_HEIGHT_CM, HOLD_SECONDS, height_cm_);
                state_     = State::HOLD;
                tick_      = 0;
                hold_tick_ = 0;
            }
            break;
        }

        // ── HOLD: full position lock 5 detik ────────────────────────────────
        case State::HOLD: {
            RC rc = computeRC(TARGET_HEIGHT_CM);
            double err = TARGET_HEIGHT_CM - height_cm_;
            sendRC(rc);

            if (tick_ % 10 == 0) {
                hold_tick_++;
                // Visual bar 0-100cm → 20 chars
                char bar[21];
                int fill = static_cast<int>((height_cm_ / 100.0) * 20);
                fill = std::clamp(fill, 0, 20);
                for (int i = 0; i < 20; i++) bar[i] = (i < fill) ? '#' : '-';
                bar[20] = '\0';
                RCLCPP_INFO(this->get_logger(),
                    "  ⏱ %d/%ds [%s] h=%.0f err=%+.0f ud=%+.0f lr=%+.0f fb=%+.0f",
                    hold_tick_, HOLD_SECONDS, bar,
                    height_cm_, err, rc.ud, rc.lr, rc.fb);
            }

            if (tick_ >= HOLD_SECONDS * 10) {
                sendStop();
                RCLCPP_INFO(this->get_logger(),
                    "[5/5] Hold done h=%.0fcm. LAND!", height_cm_);
                pub_land_->publish(std_msgs::msg::Empty());
                state_ = State::LANDING;
                tick_  = 0;
            }
            break;
        }

        // ── LANDING ──────────────────────────────────────────────────────────
        case State::LANDING:
            if (tick_ % 10 == 0)
                RCLCPP_INFO(this->get_logger(),
                    "  ↓ Landing h=%.0fcm", height_cm_);
            if (tick_ >= 30) {
                RCLCPP_INFO(this->get_logger(),
                    "Done! h=%.0fcm", height_cm_);
                RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
                state_ = State::DONE;
                timer_->cancel();
                rclcpp::shutdown();
            }
            break;

        case State::DONE: break;
        }
    }

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr       pub_takeoff_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr       pub_land_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  pub_control_;
    rclcpp::Subscription<tello_msg::msg::TelloStatus>::SharedPtr sub_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    State  state_;
    int    tick_, hold_tick_;
    double height_cm_, vgx_cms_, vgy_cms_;
    bool   height_valid_, airborne_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightNode>());
    rclcpp::shutdown();
    return 0;
}

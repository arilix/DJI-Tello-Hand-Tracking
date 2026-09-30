/**
 * teleop_node.cpp
 * Kontrol Tello via keyboard langsung dari terminal (tanpa jendela OpenCV).
 *
 * Tombol:
 *   T          → Takeoff
 *   L          → Land
 *   E          → Emergency stop
 *   W/S        → Maju / Mundur
 *   A/D        → Geser kiri / kanan
 *   Panah Atas/Bawah  → Naik / Turun
 *   Panah Kiri/Kanan  → Putar kiri / kanan (yaw)
 *   F          → Flip depan
 *   Spasi      → Berhenti (rc = 0)
 *   Q / Ctrl+C → Quit (land dulu)
 *
 * Topics yang dipublikasikan:
 *   /takeoff    std_msgs/Empty
 *   /land       std_msgs/Empty
 *   /emergency  std_msgs/Empty
 *   /control    geometry_msgs/Twist
 *   /flip       std_msgs/String
 */

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <csignal>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tello_msg/msg/tello_status.hpp>

using namespace std::chrono_literals;

// ─── ANSI helpers ─────────────────────────────────────────────────────────────
#define RST  "\033[0m"
#define BOLD "\033[1m"
#define RED  "\033[91m"
#define YLW  "\033[93m"
#define GRN  "\033[92m"
#define CYN  "\033[96m"
#define WHT  "\033[97m"
#define CLR  "\033[2J\033[H"

// ─── Terminal raw mode ────────────────────────────────────────────────────────
static struct termios g_orig_termios;

static void restoreTerminal()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

static void setRawMode()
{
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    atexit(restoreTerminal);

    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// Read one key (returns -1 if no key ready). Handles 3-byte escape sequences.
static int readKey()
{
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return -1;

    if (c == 0x1b) {  // ESC sequence (arrow keys)
        unsigned char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) != 1) return 0x1b;
        if (read(STDIN_FILENO, &seq[1], 1) != 1) return 0x1b;
        if (seq[0] == '[') {
            switch (seq[1]) {
                case 'A': return 300;  // Up
                case 'B': return 301;  // Down
                case 'C': return 302;  // Right
                case 'D': return 303;  // Left
            }
        }
        return 0x1b;
    }
    return static_cast<int>(c);
}

// ─── Node ─────────────────────────────────────────────────────────────────────
class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
    : Node("teleop_node"), speed_(50.0),
      bat_pct_(0), bat_h_cm_(0), bat_yaw_(0), bat_baro_(0),
      flying_(false)
    {
        pub_takeoff_   = create_publisher<std_msgs::msg::Empty>       ("takeoff",   1);
        pub_land_      = create_publisher<std_msgs::msg::Empty>       ("land",      1);
        pub_emergency_ = create_publisher<std_msgs::msg::Empty>       ("emergency", 1);
        pub_control_   = create_publisher<geometry_msgs::msg::Twist>  ("control",   1);
        pub_flip_      = create_publisher<std_msgs::msg::String>      ("flip",      1);

        sub_status_ = create_subscription<tello_msg::msg::TelloStatus>(
            "status", rclcpp::QoS(1),
            [this](tello_msg::msg::TelloStatus::SharedPtr m) {
                bat_pct_   = m->battery;
                bat_h_cm_  = m->distance_tof;
                bat_yaw_   = m->yaw;
                bat_baro_  = m->barometer;
            });

        // Keyboard loop di thread terpisah
        kb_thread_ = std::thread(&TeleopNode::kbLoop, this);

        printHelp();
    }

    ~TeleopNode()
    {
        running_ = false;
        if (kb_thread_.joinable()) kb_thread_.join();
        restoreTerminal();
    }

private:
    // ── Keyboard loop ────────────────────────────────────────────────────────
    void kbLoop()
    {
        setRawMode();
        auto last_status = std::chrono::steady_clock::now() - 3s;

        while (running_ && rclcpp::ok()) {
            int key = readKey();

            geometry_msgs::msg::Twist rc;
            bool send_rc  = false;
            bool redraw   = false;

            if (key == 't' || key == 'T') {
                pub_takeoff_->publish(std_msgs::msg::Empty());
                flying_ = true;
                status_msg_ = GRN "TAKEOFF dikirim" RST;
                redraw = true;
            } else if (key == 'l' || key == 'L') {
                pub_land_->publish(std_msgs::msg::Empty());
                flying_ = false;
                status_msg_ = YLW "LAND dikirim" RST;
                redraw = true;
            } else if (key == 'e' || key == 'E') {
                pub_emergency_->publish(std_msgs::msg::Empty());
                flying_ = false;
                status_msg_ = RED BOLD "EMERGENCY STOP!" RST;
                redraw = true;
            } else if (key == 'f' || key == 'F') {
                std_msgs::msg::String flip;
                flip.data = "f";
                pub_flip_->publish(flip);
                status_msg_ = CYN "FLIP!" RST;
                redraw = true;
            } else if (key == 'q' || key == 'Q') {
                status_msg_ = YLW "Keluar — mengirim land..." RST;
                printDashboard();
                pub_land_->publish(std_msgs::msg::Empty());
                std::this_thread::sleep_for(500ms);
                running_ = false;
                rclcpp::shutdown();
                break;
            } else if (key == ' ') {
                // Kirim rc nol (berhenti)
                pub_control_->publish(geometry_msgs::msg::Twist{});
                status_msg_ = WHT "STOP (rc=0)" RST;
                redraw = true;
            } else if (key != -1) {
                // Gerakan — bangun rc
                if (key == 'w' || key == 'W') { rc.linear.y  =  speed_; send_rc = true; }
                if (key == 's' || key == 'S') { rc.linear.y  = -speed_; send_rc = true; }
                if (key == 'a' || key == 'A') { rc.linear.x  = -speed_; send_rc = true; }
                if (key == 'd' || key == 'D') { rc.linear.x  =  speed_; send_rc = true; }
                if (key == 300) { rc.linear.z  =  speed_; send_rc = true; }  // Up
                if (key == 301) { rc.linear.z  = -speed_; send_rc = true; }  // Down
                if (key == 303) { rc.angular.z = -speed_; send_rc = true; }  // Yaw kiri
                if (key == 302) { rc.angular.z =  speed_; send_rc = true; }  // Yaw kanan

                if (send_rc) {
                    pub_control_->publish(rc);
                    char buf[64];
                    snprintf(buf, sizeof(buf),
                        "rc  lr=%+.0f  fb=%+.0f  ud=%+.0f  yaw=%+.0f",
                        rc.linear.x, rc.linear.y, rc.linear.z, rc.angular.z);
                    status_msg_ = std::string(CYN) + buf + RST;
                    redraw = true;
                }
            }

            // Refresh dashboard tiap 1 detik meski tidak ada keypress
            auto now = std::chrono::steady_clock::now();
            if (redraw || (now - last_status) >= 1s) {
                printDashboard();
                last_status = now;
            }

            std::this_thread::sleep_for(20ms);
        }
    }

    // ── Display ──────────────────────────────────────────────────────────────
    void printDashboard()
    {
        int pct = static_cast<int>(bat_pct_);

        // Battery bar
        const int W = 16;
        int fill = std::clamp(pct / (100 / W), 0, W);
        const char * bcol = (pct > 60) ? GRN : (pct > 25) ? YLW : RED;
        std::string bar;
        bar += bcol;
        for (int i = 0; i < W; i++) bar += (i < fill) ? "\xe2\x96\x88" : "\xe2\x96\x91";
        bar += RST;

        const char * bicon = (pct > 60) ? GRN "[PENUH]"  RST :
                             (pct > 25) ? YLW "[SEDANG]" RST :
                                          RED BOLD "[RENDAH!]" RST;

        const char * fst = flying_ ? GRN BOLD "TERBANG" RST : YLW "STANDBY" RST;

        fprintf(stdout, CLR);
        fprintf(stdout,
            CYN BOLD
            "  ╔══════════════════════════════════════════╗\n"
            "  ║         TELLO TELEOP — KONTROL           ║\n"
            "  ╚══════════════════════════════════════════╝\n"
            RST);
        fprintf(stdout,
            "  Baterai : %s [%s] " BOLD "%s%3d%%" RST "\n",
            bicon, bar.c_str(), bcol, pct);
        fprintf(stdout,
            "  Status  : %s   |  h=%dcm  yaw=%d°  baro=%dcm\n\n",
            fst, bat_h_cm_, bat_yaw_, bat_baro_);
        fprintf(stdout,
            "  " WHT "T" RST "=Takeoff  " WHT "L" RST "=Land  "
            WHT "E" RST "=Emergency  " WHT "F" RST "=Flip  "
            WHT "Q" RST "=Quit\n");
        fprintf(stdout,
            "  " WHT "W/S" RST "=Maju/Mundur  "
            WHT "A/D" RST "=Kiri/Kanan\n");
        fprintf(stdout,
            "  " WHT "↑/↓" RST "=Naik/Turun   "
            WHT "←/→" RST "=Putar Yaw  "
            WHT "Spasi" RST "=Stop\n\n");
        fprintf(stdout,
            "  Kecepatan: " BOLD "%.0f%%" RST
            "  (ubah: 1-9 × 10)\n\n",
            speed_);
        if (!status_msg_.empty())
            fprintf(stdout, "  >> %s\n", status_msg_.c_str());
        fflush(stdout);
    }

    void printHelp()
    {
        fprintf(stdout, CLR CYN BOLD
            "  ╔══════════════════════════════════════════╗\n"
            "  ║         TELLO TELEOP — KONTROL           ║\n"
            "  ╚══════════════════════════════════════════╝\n"
            RST
            "\n  Menunggu koneksi drone...\n"
            "  Tekan T untuk takeoff, Q untuk keluar.\n\n");
        fflush(stdout);
    }

    // ── Members ──────────────────────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr       pub_takeoff_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr       pub_land_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr       pub_emergency_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  pub_control_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      pub_flip_;
    rclcpp::Subscription<tello_msg::msg::TelloStatus>::SharedPtr sub_status_;

    std::thread kb_thread_;
    std::atomic<bool> running_{true};

    double speed_;
    int    bat_pct_, bat_h_cm_, bat_yaw_, bat_baro_;
    bool   flying_;
    std::string status_msg_;
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}

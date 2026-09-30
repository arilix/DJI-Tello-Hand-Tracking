/**
 * hand_control_node.cpp
 *
 * Kontrol DJI Tello lewat tracking tangan (MediaPipe C API + solvePnP).
 *
 * Keyboard (OpenCV window harus fokus):
 *   q  → takeoff
 *   e  → landing
 *   ESC → emergency land
 *
 * Tracking aktif setelah takeoff:
 *   Drone mengikuti tangan agar selalu berada di dalam kotak tengah frame.
 *   PD controller:  err X/Y (pixel) → lr/ud | err Z (PnP meter) → fb
 *
 * Topics subscribe:  /image_raw
 * Topics publish:    /takeoff  /land  /control  /hand_vision/debug_image
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>

// ─── MediaPipe C API ──────────────────────────────────────────────────────────
#include "hand_vision/mediapipe_c_api.hpp"

using namespace std::chrono_literals;

// ─── Landmark indices ─────────────────────────────────────────────────────────
enum LM {
    WRIST=0,
    THUMB_CMC=1, THUMB_MCP=2, THUMB_IP=3, THUMB_TIP=4,
    INDEX_MCP=5, INDEX_PIP=6, INDEX_DIP=7, INDEX_TIP=8,
    MIDDLE_MCP=9, MIDDLE_PIP=10, MIDDLE_DIP=11, MIDDLE_TIP=12,
    RING_MCP=13, RING_PIP=14, RING_DIP=15, RING_TIP=16,
    PINKY_MCP=17, PINKY_PIP=18, PINKY_DIP=19, PINKY_TIP=20,
    LM_COUNT=21
};

static constexpr std::array<std::pair<int,int>, 23> CONN {{
    {0,1},{1,2},{2,3},{3,4},
    {0,5},{5,6},{6,7},{7,8},
    {0,9},{9,10},{10,11},{11,12},
    {0,13},{13,14},{14,15},{15,16},
    {0,17},{17,18},{18,19},{19,20},
    {5,9},{9,13},{13,17}
}};
static constexpr std::array<int,4> PALM_MCP {INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP};

// ─── Hand detection result ────────────────────────────────────────────────────
struct Hand {
    std::array<cv::Point2f, LM_COUNT> px;
    std::array<cv::Point3f, LM_COUNT> world;
    std::string side;
    cv::Point2f palm;
};

// ─── Drone state ──────────────────────────────────────────────────────────────
enum class DroneState { IDLE, TAKING_OFF, TRACKING, LANDING };

static const char* state_str(DroneState s) {
    switch(s) {
        case DroneState::IDLE:       return "IDLE  [q=takeoff]";
        case DroneState::TAKING_OFF: return "TAKEOFF...";
        case DroneState::TRACKING:   return "TRACKING  [e=landing]";
        case DroneState::LANDING:    return "LANDING...";
    }
    return "?";
}

// ─── Simple PD controller ─────────────────────────────────────────────────────
struct PD {
    double kp, kd, max_out, deadband;
    double prev_err{0};

    double compute(double err, double dt) {
        if (std::abs(err) < deadband) { prev_err = 0; return 0; }
        double out = kp * err + kd * (err - prev_err) / std::max(dt, 1e-3);
        prev_err = err;
        return std::clamp(out, -max_out, max_out);
    }
    void reset() { prev_err = 0; }
};

// ─── UI helper ────────────────────────────────────────────────────────────────
static void label(cv::Mat& f, const std::string& s, cv::Point pt,
                  cv::Scalar col, double sc = 0.48)
{
    int base;
    auto sz = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base);
    cv::rectangle(f, {pt.x-2, pt.y-sz.height-2}, {pt.x+sz.width+4, pt.y+base+2},
                  {0,0,0}, -1);
    cv::putText(f, s, pt, cv::FONT_HERSHEY_SIMPLEX, sc, col, 1, cv::LINE_AA);
}

// ─── Node ─────────────────────────────────────────────────────────────────────
class HandControlNode : public rclcpp::Node
{
public:
    HandControlNode() : Node("hand_control_node")
    {
        // ── Params ────────────────────────────────────────────────────────────
        declare_parameter("hand_model_path",  "");
        declare_parameter("camera_topic",     "/image_raw");
        declare_parameter("target_distance",  0.30);   // meter
        declare_parameter("box_px",           130);    // tracking box half-size px
        declare_parameter("kp_lateral",       35.0);   // gain: norm-error → %speed
        declare_parameter("kd_lateral",       0.0);
        declare_parameter("kp_depth",          0.0);   // 0 = FB disabled by default
        declare_parameter("kd_depth",          0.0);
        declare_parameter("max_speed_lat",    40.0);   // %
        declare_parameter("max_speed_fb",     28.0);   // %
        declare_parameter("deadband_px",      22.0);   // pixel
        declare_parameter("deadband_z",       0.08);   // normalized fraction
        declare_parameter("lr_sign",          1);      // +1 or -1 to flip LR direction
        declare_parameter("rc_alpha",         0.45);   // EMA alpha for RC output
        declare_parameter("hand_real_width",  0.08);   // meter, avg palm width
        declare_parameter("takeoff_wait_ms",  4500);   // ms tunggu setelah takeoff
        declare_parameter("land_wait_ms",     3000);   // ms tunggu setelah land
        declare_parameter("smoothing_alpha",  0.35);
        declare_parameter("camera_matrix",
            std::vector<double>{924,0,480, 0,924,360, 0,0,1});
        declare_parameter("dist_coeffs",
            std::vector<double>{0,0,0,0,0});
        declare_parameter("min_detect_conf",  0.60f);
        declare_parameter("min_track_conf",   0.50f);

        model_path_    = get_parameter("hand_model_path").as_string();
        camera_topic_  = get_parameter("camera_topic").as_string();
        target_z_      = get_parameter("target_distance").as_double();
        box_px_        = get_parameter("box_px").as_int();
        max_lat_       = get_parameter("max_speed_lat").as_double();
        max_fb_        = get_parameter("max_speed_fb").as_double();
        db_px_         = get_parameter("deadband_px").as_double();
        db_z_          = get_parameter("deadband_z").as_double();
        alpha_         = get_parameter("smoothing_alpha").as_double();
        lr_sign_       = get_parameter("lr_sign").as_int();
        rc_alpha_      = get_parameter("rc_alpha").as_double();
        hand_w_m_      = get_parameter("hand_real_width").as_double();
        kp_lat_        = get_parameter("kp_lateral").as_double();
        kp_fb_         = get_parameter("kp_depth").as_double();
        takeoff_wait_  = std::chrono::milliseconds(get_parameter("takeoff_wait_ms").as_int());
        land_wait_     = std::chrono::milliseconds(get_parameter("land_wait_ms").as_int());

        auto cam  = get_parameter("camera_matrix").as_double_array();
        auto dist = get_parameter("dist_coeffs").as_double_array();
        K_ = (cam.size()==9)
             ? cv::Mat(3,3,CV_64F,cam.data()).clone()
             : (cv::Mat_<double>(3,3) << 924,0,480, 0,924,360, 0,0,1);
        D_ = cv::Mat((int)dist.size(),1,CV_64F,dist.data()).clone();

        // ── MediaPipe ─────────────────────────────────────────────────────────
        if (!initMediaPipe()) { rclcpp::shutdown(); return; }

        // ── Publishers ────────────────────────────────────────────────────────
        pub_takeoff_ = create_publisher<std_msgs::msg::Empty>("takeoff", 1);
        pub_land_    = create_publisher<std_msgs::msg::Empty>("land",    1);
        pub_ctrl_    = create_publisher<geometry_msgs::msg::Twist>("control", 1);
        pub_img_     = create_publisher<sensor_msgs::msg::Image>("/hand_vision/debug_image", 5);

        // ── Subscriber ────────────────────────────────────────────────────────
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(
                   rmw_qos_profile_sensor_data));
        sub_img_ = create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, qos,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg){ imageCb(msg); });

        // ── Keyboard timer (30Hz, main-thread-safe) ───────────────────────────
        kb_timer_ = create_wall_timer(33ms, [this]{ keyboardTick(); });

        RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
        RCLCPP_INFO(get_logger(), "  Tello Hand Control C++ — PnP Tracking");
        RCLCPP_INFO(get_logger(), "  model      : %s", model_path_.c_str());
        RCLCPP_INFO(get_logger(), "  target_dist: %.2fm | box: %dpx | lr_sign: %d",
                    target_z_, box_px_, lr_sign_);
        RCLCPP_INFO(get_logger(), "  kp_lat=%.1f kp_fb=%.1f  rc_alpha=%.2f",
                    get_parameter("kp_lateral").as_double(),
                    get_parameter("kp_depth").as_double(), rc_alpha_);
        RCLCPP_INFO(get_logger(), "  Keyboard: q=takeoff  e=landing  ESC=emergency");
        RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
    }

    ~HandControlNode()
    {
        running_ = false;
        if (mp_handle_) {
            char* err = nullptr;
            MpHandLandmarkerClose(mp_handle_, &err);
            if (err) MpErrorFree(err);
        }
        cv::destroyAllWindows();
    }

private:
    // ── MediaPipe init ────────────────────────────────────────────────────────
    bool initMediaPipe()
    {
        if (model_path_.empty()) {
            RCLCPP_FATAL(get_logger(),
                "hand_model_path belum diset!\n"
                "  -p hand_model_path:=/path/to/hand_landmarker.task");
            return false;
        }
        MpHandLandmarkerOptions opts{};
        opts.base_options.model_asset_path         = model_path_.c_str();
        opts.base_options.model_asset_buffer       = nullptr;
        opts.base_options.model_asset_buffer_count = 0;
        opts.base_options.delegate                 = 0;
        opts.base_options.host_environment         = 0;
        opts.base_options.host_system              = 0;
        opts.base_options.host_version             = nullptr;
        opts.base_options.ca_bundle_path           = nullptr;
        opts.running_mode                   = MP_IMAGE;
        opts.num_hands                      = 1;
        opts.min_hand_detection_confidence  = static_cast<float>(
            get_parameter("min_detect_conf").as_double());
        opts.min_hand_presence_confidence   = opts.min_hand_detection_confidence;
        opts.min_tracking_confidence        = static_cast<float>(
            get_parameter("min_track_conf").as_double());
        opts.result_callback                = nullptr;

        char* err = nullptr;
        int st = MpHandLandmarkerCreate(&opts, &mp_handle_, &err);
        if (st != 0 || !mp_handle_) {
            RCLCPP_FATAL(get_logger(), "MpHandLandmarkerCreate failed: %s",
                         err ? err : "(no msg)");
            if (err) MpErrorFree(err);
            return false;
        }
        RCLCPP_INFO(get_logger(), "MediaPipe HandLandmarker C++ OK");
        return true;
    }

    // ── Image callback ────────────────────────────────────────────────────────
    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv::Mat bgr;
        try { bgr = cv_bridge::toCvCopy(msg, "bgr8")->image; }
        catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
            return;
        }
        processFrame(bgr, msg->header.stamp);
    }

    // ── Frame processing ──────────────────────────────────────────────────────
    void processFrame(const cv::Mat& bgr, const rclcpp::Time& stamp)
    {
        const int W = bgr.cols, H = bgr.rows;

        // FPS
        {
            std::lock_guard<std::mutex> lk(disp_mutex_);
            double t = nowSec();
            fps_buf_.push_back(t);
            while (!fps_buf_.empty() && fps_buf_.front() < t - 1.0)
                fps_buf_.pop_front();
        }

        prev_t_ = nowSec();

        // ── Deteksi pada frame ASLI (koordinat benar untuk kontrol) ─────────
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        Hand hand;
        bool detected = detectHand(rgb, hand);

        // ── PnP pose ──────────────────────────────────────────────────────────
        cv::Vec3d tvec_smooth;
        bool pose_valid = false;
        if (detected) {
            std::vector<cv::Point3f> obj;
            std::vector<cv::Point2f> img;
            for (int i : PALM_MCP) { obj.push_back(hand.world[i]); img.push_back(hand.px[i]); }
            cv::Mat rv, tv;
            if (cv::solvePnP(obj, img, K_, D_, rv, tv, false, cv::SOLVEPNP_IPPE)) {
                cv::Vec3d tv3(tv);
                if (pose_prev_valid_) {
                    tvec_smooth_ = alpha_ * tv3 + (1.0-alpha_) * tvec_smooth_;
                } else {
                    tvec_smooth_ = tv3;
                    pose_prev_valid_ = true;
                }
                tvec_smooth = tvec_smooth_;
                pose_valid  = true;
            }
        } else {
            pose_prev_valid_ = false;
        }

        // ── Control ───────────────────────────────────────────────────────────
        DroneState cur_state;
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            cur_state = state_;
        }

        geometry_msgs::msg::Twist rc;
        bool send_rc = false;

        if (cur_state == DroneState::TRACKING) {
            if (detected) {
                double cx = W / 2.0, cy = H / 2.0;
                double ex = (hand.palm.x - cx) / (W / 2.0);  // [-1, 1]
                double ey = (hand.palm.y - cy) / (H / 2.0);
                double db_norm = db_px_ / (W / 2.0);

                // Pure P — zero inside deadband, no EMA residual
                double lr = (std::abs(ex) > db_norm)
                            ? std::clamp(lr_sign_ * kp_lat_ * ex, -max_lat_, max_lat_)
                            : 0.0;
                double ud = (std::abs(ey) > db_norm)
                            ? std::clamp(-kp_lat_ * ey, -max_lat_, max_lat_)
                            : 0.0;

                // FB: hand pixel size as distance proxy
                double hand_size_px = cv::norm(hand.px[INDEX_MCP] - hand.px[PINKY_MCP]);
                double ref_size_px  = hand_w_m_ * K_.at<double>(0,0) / target_z_;
                double ez_size = (ref_size_px - hand_size_px) / ref_size_px;
                double fb = (std::abs(ez_size) > db_z_)
                            ? std::clamp(kp_fb_ * ez_size, -max_fb_, max_fb_)
                            : 0.0;

                rc.linear.x  = lr;
                rc.linear.y  = fb;
                rc.linear.z  = ud;
                rc.angular.z = 0;

                if (++log_counter_ % 30 == 0) {
                    RCLCPP_INFO(get_logger(),
                        "TRACK | ex:%+.2f ey:%+.2f ez:%+.2f hand:%.0f ref:%.0f"
                        " | LR:%+.0f UD:%+.0f FB:%+.0f",
                        ex, ey, ez_size, hand_size_px, ref_size_px,
                        lr, ud, fb);
                }
                send_rc = true;
            } else {
                pub_ctrl_->publish(geometry_msgs::msg::Twist{});
            }
        }

        if (send_rc) pub_ctrl_->publish(rc);

        // ── Gambar UI — flip frame untuk display natural (kiri = kiri) ────────
        cv::Mat out;
        cv::flip(bgr, out, 1);  // flip hanya untuk tampilan

        // Mirror landmark X untuk digambar di frame yang sudah di-flip
        std::array<cv::Point2f, LM_COUNT> px_disp = hand.px;
        cv::Point2f palm_disp = hand.palm;
        if (detected) {
            for (auto& p : px_disp)  p.x = W - 1 - p.x;
            palm_disp.x = W - 1 - hand.palm.x;
            drawHand(out, px_disp);
        }

        // Tracking box (pakai koordinat display)
        drawBox(out, detected ? &palm_disp : nullptr, W, H, cur_state);

        // HUD
        float fps;
        {
            std::lock_guard<std::mutex> lk(disp_mutex_);
            fps = static_cast<float>(fps_buf_.size());
        }

        // Status bar
        cv::Scalar st_col;
        switch(cur_state) {
            case DroneState::IDLE:       st_col = {110,110,110}; break;
            case DroneState::TAKING_OFF: st_col = {220,200,0};   break;
            case DroneState::TRACKING:   st_col = {0,220,0};     break;
            case DroneState::LANDING:    st_col = {0,165,255};   break;
        }
        label(out, state_str(cur_state), {8, 28}, st_col, 0.60);

        char buf[128];
        std::snprintf(buf,sizeof(buf),"FPS:%.0f",fps);
        label(out, buf, {W-70, 20}, {255,255,255}, 0.45);

        if (detected && pose_valid) {
            std::snprintf(buf,sizeof(buf),"Dist:%.2fm  Z:%.2f  X:%.2f  Y:%.2f",
                cv::norm(tvec_smooth), tvec_smooth[2], tvec_smooth[0], tvec_smooth[1]);
            label(out, buf, {8,52}, {255,255,255}, 0.40);
        }

        if (send_rc) {
            std::snprintf(buf,sizeof(buf),"LR:%+.0f  FB:%+.0f  UD:%+.0f",
                rc.linear.x, rc.linear.y, rc.linear.z);
            label(out, buf, {8, H-14}, {220,200,0}, 0.42);
        }

        // Publish debug image
        try {
            auto img_msg = cv_bridge::CvImage(
                std_msgs::msg::Header{}, "bgr8", out).toImageMsg();
            pub_img_->publish(*img_msg);
        } catch(...) {}

        // Simpan untuk display di keyboard timer
        {
            std::lock_guard<std::mutex> lk(disp_mutex_);
            out.copyTo(disp_frame_);
            disp_fresh_ = true;
        }

        (void)stamp;
    }

    // ── MediaPipe detect ──────────────────────────────────────────────────────
    bool detectHand(const cv::Mat& rgb, Hand& out)
    {
        if (!mp_handle_) return false;
        void* mp_img = nullptr;
        char* err    = nullptr;
        int st = MpImageCreateFromUint8Data(
            1, rgb.cols, rgb.rows,
            const_cast<uint8_t*>(rgb.data),
            rgb.cols * rgb.rows * rgb.channels(),
            &mp_img, &err);
        if (st!=0 || !mp_img) { if(err) MpErrorFree(err); return false; }

        MpHandLandmarkerResult res{};
        err = nullptr;
        st = MpHandLandmarkerDetectImage(mp_handle_, mp_img, nullptr, &res, &err);
        MpImageFree(mp_img);
        if (st!=0) { if(err) MpErrorFree(err); return false; }

        bool found = (res.hand_landmarks_count > 0 &&
                      res.hand_landmarks != nullptr &&
                      res.hand_landmarks[0].landmarks_count >= LM_COUNT);
        if (found) {
            const float W = static_cast<float>(rgb.cols);
            const float H = static_cast<float>(rgb.rows);
            const auto& ln = res.hand_landmarks[0];
            const auto& lw = res.hand_world_landmarks[0];
            for (int i = 0; i < LM_COUNT; ++i) {
                out.px[i]    = { ln.landmarks[i].x * W, ln.landmarks[i].y * H };
                out.world[i] = { lw.landmarks[i].x, lw.landmarks[i].y, lw.landmarks[i].z };
            }
            if (res.handedness_count > 0 &&
                res.handedness[0].categories_count > 0 &&
                res.handedness[0].categories[0].category_name)
                out.side = res.handedness[0].categories[0].category_name;
            else
                out.side = "?";

            cv::Point2f sum{0,0};
            sum += out.px[WRIST];
            for (int i : PALM_MCP) sum += out.px[i];
            out.palm = sum * (1.0f / 5.0f);
        }
        MpHandLandmarkerCloseResult(&res);
        return found;
    }

    // ── Draw ─────────────────────────────────────────────────────────────────
    void drawHand(cv::Mat& f, const std::array<cv::Point2f, LM_COUNT>& px)
    {
        for (auto [a,b] : CONN)
            cv::line(f, cvPt(px[a]), cvPt(px[b]), {100,100,100}, 2, cv::LINE_AA);

        const std::array<int,5> TIPS{THUMB_TIP,INDEX_TIP,MIDDLE_TIP,RING_TIP,PINKY_TIP};
        for (int i = 0; i < LM_COUNT; ++i) {
            bool tip = std::find(TIPS.begin(),TIPS.end(),i) != TIPS.end();
            cv::Scalar col = (i==WRIST) ? cv::Scalar(0,220,220)
                           : tip        ? cv::Scalar(0,220,0)
                                        : cv::Scalar(220,220,220);
            cv::circle(f, cvPt(px[i]), tip?5:3, col, -1, cv::LINE_AA);
        }
    }

    void drawBox(cv::Mat& f, const cv::Point2f* palm,
                 int W, int H, DroneState st)
    {
        int cx = W/2, cy = H/2, bp = box_px_, arm = 22;
        bool locked = (palm != nullptr && st == DroneState::TRACKING &&
                       std::abs(palm->x - cx) < bp &&
                       std::abs(palm->y - cy) < bp);
        cv::Scalar col = locked ? cv::Scalar(0,220,0) : cv::Scalar(40,40,220);

        for (auto [dx,dy] : std::array<std::pair<int,int>,4>{{
            {-bp,-bp},{bp,-bp},{bp,bp},{-bp,bp}}})
        {
            cv::Point c{cx+dx, cy+dy};
            cv::line(f, c, {c.x+(dx<0?arm:-arm), c.y}, col, 2, cv::LINE_AA);
            cv::line(f, c, {c.x, c.y+(dy<0?arm:-arm)}, col, 2, cv::LINE_AA);
        }
        cv::circle(f, {cx,cy}, 4, col, -1, cv::LINE_AA);

        if (palm && !locked && st==DroneState::TRACKING)
            cv::arrowedLine(f, cvPt(*palm), {cx,cy}, {0,165,255}, 2,
                            cv::LINE_AA, 0, 0.25);
    }

    // ── Keyboard timer (30Hz, hanya dari sini cv::imshow dipanggil) ──────────
    void keyboardTick()
    {
        // Display
        {
            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lk(disp_mutex_);
                if (!disp_frame_.empty()) {
                    disp_frame_.copyTo(frame);
                    disp_fresh_ = false;
                }
            }
            if (frame.empty()) {
                frame = cv::Mat::zeros(480, 640, CV_8UC3);
                label(frame, "Waiting for /image_raw ...", {60, 240},
                      {160, 160, 160}, 0.70);
                label(frame, "q=takeoff  e=land  ESC=emergency", {100, 290},
                      {100, 100, 100}, 0.50);
            }
            cv::imshow("Tello Hand Control", frame);
        }

        int key = cv::waitKey(1);

        // State mutex: handle key + SELALU jalankan state transitions
        std::lock_guard<std::mutex> lk(state_mutex_);

        if (key >= 0) {
            switch (key & 0xFF) {
            case 'q': case 'Q':
                if (state_ == DroneState::IDLE) {
                    RCLCPP_INFO(get_logger(), "[q] TAKEOFF");
                    pub_takeoff_->publish(std_msgs::msg::Empty{});
                    state_    = DroneState::TAKING_OFF;
                    state_ts_ = std::chrono::steady_clock::now();
                }
                break;
            case 'e': case 'E':
                if (state_ == DroneState::TRACKING ||
                    state_ == DroneState::TAKING_OFF) {
                    RCLCPP_INFO(get_logger(), "[e] LANDING");
                    stopAndLand();
                }
                break;
            case 27:  // ESC
                RCLCPP_WARN(get_logger(), "[ESC] EMERGENCY LAND");
                stopAndLand();
                break;
            }
        }

        // Selalu cek timeout transisi — TIDAK boleh skip saat tidak ada key
        tickStateTransitions();
    }

    void tickStateTransitions()
    {
        // state_mutex_ sudah dipegang oleh caller
        auto elapsed = std::chrono::steady_clock::now() - state_ts_;
        switch (state_) {
        case DroneState::TAKING_OFF:
            if (elapsed >= takeoff_wait_) {
                state_ = DroneState::TRACKING;
                RCLCPP_INFO(get_logger(), "TRACKING aktif");
            }
            break;
        case DroneState::LANDING:
            if (elapsed >= land_wait_) {
                state_ = DroneState::IDLE;
                RCLCPP_INFO(get_logger(), "Landed — IDLE");
            }
            break;
        default: break;
        }
    }

    void stopAndLand()
    {
        // state_mutex_ sudah dipegang
        pub_ctrl_->publish(geometry_msgs::msg::Twist{});
        pub_land_->publish(std_msgs::msg::Empty{});
        state_    = DroneState::LANDING;
        state_ts_ = std::chrono::steady_clock::now();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    static cv::Point cvPt(const cv::Point2f& p)
    { return {(int)p.x, (int)p.y}; }

    static double nowSec() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // ── Members ───────────────────────────────────────────────────────────────
    std::string model_path_, camera_topic_;
    double      target_z_;
    int         box_px_, lr_sign_;
    double      max_lat_, max_fb_, db_px_, db_z_, alpha_, rc_alpha_, hand_w_m_;
    double      kp_lat_, kp_fb_;
    std::chrono::milliseconds takeoff_wait_, land_wait_;
    cv::Mat     K_, D_;

    void*       mp_handle_{nullptr};


    // Pose smoothing
    bool        pose_prev_valid_{false};
    cv::Vec3d   tvec_smooth_;
    double      prev_t_{0};
    int         log_counter_{0};

    // State machine
    std::mutex  state_mutex_;
    DroneState  state_{DroneState::IDLE};
    std::chrono::steady_clock::time_point state_ts_{};

    // Display buffer (written by image cb, read by kb timer)
    std::mutex  disp_mutex_;
    cv::Mat     disp_frame_;
    bool        disp_fresh_{false};
    std::deque<double> fps_buf_;

    std::atomic<bool> running_{true};

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr         pub_takeoff_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr         pub_land_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_ctrl_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      pub_img_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr   sub_img_;
    rclcpp::TimerBase::SharedPtr                               kb_timer_;
};

// ─── main ────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

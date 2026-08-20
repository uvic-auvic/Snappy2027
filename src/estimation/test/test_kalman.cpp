// Offline replay test for the KalmanFilter (no ROS required).
//
// Replays a recorded run from the CSVs the state_estimator node writes
// (imu2_*.csv, imu1_*.csv, depth_*.csv, dvl_*.csv) through the exact same
// KalmanFilter configuration and callback logic as state_estimator.cpp,
// writes kalman_replay_<run>.csv in the same format as kalman_<run>.csv,
// and compares the two row by row.
//
// Usage:   test_kalman [path/to/state_estimator_outputs | path/to/run_dir] [dvl_offset_sec]
//          (default folder: ./state_estimator_outputs; default offset: auto)
// Runs live in per-run subdirectories named by timestamp
// (state_estimator_outputs/<run_id>/); with no run dir given, the newest
// run is replayed. Pass a specific run's directory to replay an older one.
//
// ---- Timing reconstruction ----
// The recorded CSVs do not store message ARRIVAL times for every stream, so
// the replay reconstructs them:
//  - IMU2 rows carry sensor timestamps and drive predict(); the recorded
//    kalman CSV has exactly one row per IMU2 row, so IMU2 is the replay clock.
//  - DVL rows carry driver timestamps in a different epoch, so the offset
//    between the DVL and IMU2 clocks is unknown. The test estimates it by
//    replaying with a range of candidate offsets and keeping the one whose
//    velocity best matches the reference output (replaying is cheap).
//    Pass an explicit offset in seconds as the 2nd argument to skip the scan.
//  - IMU1 rows (if any) are aligned first-message-to-first-IMU2-message.
//  - Depth rows have NO timestamps. The arrival of the FIRST depth message is
//    recovered from the reference kalman CSV: the first row whose state moved
//    off the pre-init default (zeros + identity quaternion) marks frame
//    initialization, which only happens once depth has arrived. Later rows
//    are anchored to the reference by VALUE: R_depth is tiny, so every depth
//    update snaps the reference z to ~the measured value, and each z-jump is
//    one depth row landing. Rows that made no visible jump are spread evenly
//    between the surrounding anchors.
// Because DVL/depth arrival times are reconstructed rather than recorded, the
// replay matches the recorded output closely but not bit-for-bit.

#include "snappy_estimation/kalman.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// One parsed CSV row: nanosecond timestamp (0 for files without one) + values.
// Timestamps are kept as int64 — DVL stamps (~1.7e18 ns) lose precision in a double.
struct CsvRow {
    long long t_ns = 0;
    std::vector<double> v;
};

static std::vector<CsvRow> loadRows(const fs::path& path, bool has_timestamp)
{
    std::vector<CsvRow> rows;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "ERROR: could not open " << path << std::endl;
        return rows;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        CsvRow row;
        bool first = true;
        while (std::getline(ss, cell, ',')) {
            if (first && has_timestamp) {
                row.t_ns = std::stoll(cell);
            } else {
                row.v.push_back(std::stod(cell));
            }
            first = false;
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

// Mirrors StateEstimator (state_estimator.cpp) with the ROS pieces removed.
// Setup values and callback bodies are kept identical on purpose — if you tune
// the node, tune this the same way (or better: move shared config to one place).
class OfflineStateEstimator
{
public:
    KalmanFilter kf;

    // One row per IMU2 message: t_ns, pos(3), vel(3), quat(w,x,y,z) —
    // the same thing the node writes to its kalman_*.csv.
    std::vector<CsvRow> output;

    // First post-init quaternion from the reference kalman CSV, used to
    // recover the node's mission-frame alignment (see try_initialize).
    Quaterniond ref_q0_;
    bool have_ref_q0_ = false;
    OfflineStateEstimator()
    {
        accel_bias_init_ << 0.0, 0.0, 0.0;

        // Default state before frame initialization fires (filter not yet active)
        VectorXd x0 = VectorXd::Zero(13);
        x0(6) = 1.0;  // identity quaternion [w, x, y, z]
        x0.segment<3>(10) = accel_bias_init_;

        // Error-state covariance: [δp(3), δv(3), δθ(3), δb_a(3)] = 12 dims
        P_init_ = MatrixXd::Identity(12, 12);
        P_init_.block<3,3>(0,0) *= 1;   // position uncertainty of initial guess
        P_init_.block<3,3>(3,3) *= 0.01;   // velocity uncertainty of initial guess
        P_init_.block<3,3>(6,6) *= 0.01;  // orientation error uncertainty of initial guess
        P_init_.block<3,3>(9,9) *= 0.01;  // accel bias uncertainty of initial guess

        // Q: 6x6 — two active noise sources: IMU2 accel noise and accel bias random walk
        MatrixXd Q_init = MatrixXd::Zero(6, 6);
        Q_init.block<3,3>(0,0) = Matrix3d::Identity() * 0.1;    // accel noise → velocity growth
        Q_init.block<3,3>(3,3) = Matrix3d::Identity() * 1e-6;   // accel bias random walk
        kf.setProcessNoise(Q_init);

        MatrixXd R_imu1_init = MatrixXd::Identity(3, 3) * 1.0;   // IMU1 update noise (low trust)
        MatrixXd R_depth_init = MatrixXd::Identity(1, 1) * 0.01; // 0.05 // depth sensor noise
        MatrixXd R_dvl_vel = MatrixXd::Identity(3, 3) * 0.09;    // DVL velocity noise, high trust

        kf.setIMU1MeasurementNoise(R_imu1_init);
        kf.setDepthMeasurementNoise(R_depth_init);
        kf.setDVLVelocityMeasurementNoise(R_dvl_vel);

        // IMU2 is mounted upside-down: 180° around y
        q_imu2_to_body_node_ = Quaterniond(0, 0, 1, 0);  // (w, x, y, z)
        kf.setIMU2ToBodyRotation(q_imu2_to_body_node_);

        kf.setIMU1ToBodyRotation(Quaterniond(0, 1, 0, 0));

        kf.reset(x0, P_init_);
    }

    // imu1 CSV columns after timestamp: accel_x..z, gyro_x..z
    void imu1_callback(const CsvRow& row)
    {
        const double now_sec = row.t_ns * 1e-9;
        if (!imu1_initialized_) {
            imu1_initialized_ = true;
            last_time_imu1_sec_ = now_sec;
        }
        const double dt = now_sec - last_time_imu1_sec_;
        last_time_imu1_sec_ = now_sec;
        if (dt > 0.001 && frame_initialized_)
        {
            Vector3d accel(row.v[0], row.v[1], row.v[2]);
            Vector3d gyro(row.v[3], row.v[4], row.v[5]);
            kf.updateIMU1(accel, gyro, dt);
        }
    }

    // imu2 CSV columns after timestamp: accel_x..z, quat_x, quat_y, quat_z, quat_w.
    // Records one kalman output row per message, exactly like the node.
    void imu2_callback(const CsvRow& row)
    {
        const double now_sec = row.t_ns * 1e-9;
        if (!imu2_initialized_) {
            imu2_initialized_ = true;
            last_time_imu2_sec_ = now_sec;
        }

        if (!init_imu2_ready_) {
            init_imu2_quat_ = Quaterniond(row.v[6], row.v[3], row.v[4], row.v[5]);  // (w, x, y, z)
            init_imu2_ready_ = true;
            try_initialize();
            // In the live node depth arrives last, so the depth message that
            // completes frame init falls through to updateDepth(). Here the
            // buffered depth event was consumed before this row (it could not
            // init without the quat), so apply it now the filter exists.
            if (frame_initialized_ && init_depth_ready_) kf.updateDepth(init_depth_);
        }

        const double dt = now_sec - last_time_imu2_sec_;
        last_time_imu2_sec_ = now_sec;

        if (dt > 0.001 && frame_initialized_)
        {
            Vector3d accel(row.v[0], row.v[1], row.v[2]);
            Quaterniond quat(row.v[6], row.v[3], row.v[4], row.v[5]);
            kf.predict(accel, quat, dt);
        }

        CsvRow out;
        out.t_ns = row.t_ns;
        out.v = { kf.getPosition().x(),    kf.getPosition().y(),    kf.getPosition().z(),
                  kf.getVelocity().x(),    kf.getVelocity().y(),    kf.getVelocity().z(),
                  kf.getOrientation().w(), kf.getOrientation().x(),
                  kf.getOrientation().y(), kf.getOrientation().z() };
        output.push_back(std::move(out));
    }

    // The node receives depth as std_msgs::Float32, so the caller passes a float
    // to get the same rounded value the filter saw in the recorded run.
    void depth_callback(float depth_data)
    {
        if (!init_depth_ready_) {
            init_depth_ = depth_data;
            init_depth_ready_ = true;
            try_initialize();
        }
        if (frame_initialized_)
        {
            kf.updateDepth(depth_data);
        }
    }

    // dvl CSV columns after timestamp: v_x, v_y, v_z (body frame)
    void dvl_callback(const CsvRow& row)
    {
        if (!frame_initialized_) return;
        const float epsilon = 1e-6f;
        // The node writes the ALREADY-TRANSFORMED v_body (-x, y, -z applied) to
        // the dvl CSV, so replay it verbatim. (Recordings made before the node's
        // z-sign fix would need -z here — those predate the mission frame and
        // fail the comparison anyway.)
        Vector3d v_body(row.v[0], row.v[1], row.v[2]);
        //if (!v_body.allFinite()) return;
        
        // Should test this next pool test, may crate better results hard to tell
        if (abs(v_body.x()) < epsilon && abs(v_body.y()) < epsilon && abs(v_body.z()) < epsilon) return; // skip bad samples
        

        kf.updateDVLVelocity(v_body);
    }

private:
    void try_initialize()
    {
        if (frame_initialized_ || !init_depth_ready_ || !init_imu2_ready_) return;

        double z0 = init_depth_;

        // Same frame math as state_estimator.cpp: fix both sides of the Xsens
        // quaternion (ENU world reference + upside-down mounting) before use.
        Quaterniond q_body0 = (KalmanFilter::q_enu_to_world_ * init_imu2_quat_.normalized()
                               * q_imu2_to_body_node_.conjugate()).normalized();

        const double yaw0 = std::atan2(
            2.0 * (q_body0.w() * q_body0.z() + q_body0.x() * q_body0.y()),
            1.0 - 2.0 * (q_body0.y() * q_body0.y() + q_body0.z() * q_body0.z()));

        

        // Mission frame (same as state_estimator.cpp): Rz(-yaw0) cancels the
        // starting NED yaw, so world +x = initial heading, +y right of it.
        // Startup yaw is exactly 0; roll/pitch as measured.
        //
        // The live node aligns from its very FIRST IMU2 message, which never
        // reaches the CSV (pre-init rows are skipped), so aligning from our
        // first available row leaves a small constant yaw offset (~0.4 deg
        // observed). When the reference's first row is already post-init,
        // recover the node's ACTUAL alignment from it instead: the filter's
        // attitude is slaved to q_ned_to_world * (ENU/mount-fixed IMU2 quat),
        // so the yaw of q_ref0 * q_body0^-1 is the node's q_align.
        double align_yaw = -yaw0;
        if (have_ref_q0_) {
            const Quaterniond q_off = (ref_q0_ * q_body0.conjugate()).normalized();
            align_yaw = std::atan2(
                2.0 * (q_off.w() * q_off.z() + q_off.x() * q_off.y()),
                1.0 - 2.0 * (q_off.y() * q_off.y() + q_off.z() * q_off.z()));
        }
        const Quaterniond q_align(AngleAxisd(align_yaw, Vector3d::UnitZ()));
        kf.q_ned_to_world_ = q_align;
        Quaterniond q0 = (q_align * q_body0).normalized();

        VectorXd x0 = VectorXd::Zero(13);
        x0(2) = z0;
        x0(6) = q0.w();
        x0(7) = q0.x();
        x0(8) = q0.y();
        x0(9) = q0.z();
        x0.segment<3>(10) = accel_bias_init_;

        kf.reset(x0, P_init_);
        frame_initialized_ = true;
    }

    double last_time_imu1_sec_ = 0.0;
    double last_time_imu2_sec_ = 0.0;
    bool imu1_initialized_ = false;
    bool imu2_initialized_ = false;
    bool frame_initialized_ = false;
    bool init_depth_ready_  = false;
    bool init_imu2_ready_   = false;
    double init_depth_ = 0.0;
    Quaterniond init_imu2_quat_;
    Quaterniond q_imu2_to_body_node_;
    Vector3d accel_bias_init_ = Vector3d::Zero();
    MatrixXd P_init_;
};

// A sensor message in the merged replay timeline. Time is seconds relative to
// the start of the run (= first IMU2 message).
struct Event {
    double t;
    int kind;    // processed in this order at equal times: 0=depth, 1=dvl, 2=imu1, 3=imu2
    size_t idx;  // row index into that stream's CSV
};

struct ReplayInputs {
    std::vector<CsvRow> imu1, imu2, dvl;
    std::vector<CsvRow> depth;
    std::vector<double> depth_times;  // reconstructed arrival times (rel. seconds)
    Quaterniond ref_q0;               // reference row-1 quaternion (frame anchor)
    bool have_ref_q0 = false;
};

// Replays the whole run with the given DVL clock offset and returns the
// filter output (one row per IMU2 message).
static std::vector<CsvRow> runReplay(const ReplayInputs& in, double dvl_offset_sec)
{
    const double t0_imu2 = in.imu2.front().t_ns * 1e-9;

    std::vector<Event> events;
    events.reserve(in.imu1.size() + in.imu2.size() + in.dvl.size() + in.depth.size());

    for (size_t i = 0; i < in.imu2.size(); ++i)
        events.push_back({in.imu2[i].t_ns * 1e-9 - t0_imu2, 3, i});
    for (size_t i = 0; i < in.dvl.size(); ++i)
        events.push_back({(in.dvl[i].t_ns - in.dvl.front().t_ns) * 1e-9 + dvl_offset_sec, 1, i});
    for (size_t i = 0; i < in.imu1.size(); ++i)
        events.push_back({(in.imu1[i].t_ns - in.imu1.front().t_ns) * 1e-9, 2, i});
    for (size_t i = 0; i < in.depth_times.size(); ++i)
        events.push_back({in.depth_times[i], 0, i});

    std::sort(events.begin(), events.end(), [](const Event& a, const Event& b) {
        if (a.t != b.t) return a.t < b.t;
        if (a.kind != b.kind) return a.kind < b.kind;
        return a.idx < b.idx;
    });

    OfflineStateEstimator est;
    est.ref_q0_ = in.ref_q0;
    est.have_ref_q0_ = in.have_ref_q0;
    for (const Event& e : events) {
        switch (e.kind) {
            case 0: est.depth_callback(static_cast<float>(in.depth[e.idx].v[0])); break;
            case 1: est.dvl_callback(in.dvl[e.idx]);   break;
            case 2: est.imu1_callback(in.imu1[e.idx]); break;
            case 3: est.imu2_callback(in.imu2[e.idx]); break;
        }
    }
    return est.output;
}

struct CompareStats {
    double rms_dp = 0, max_dp = 0;      // position error [m]
    double rms_dv = 0, max_dv = 0;      // velocity error [m/s]
    double rms_dang = 0, max_dang = 0;  // attitude error [deg]
    size_t max_dp_row = 0;              // 1-based row of worst position error
};

static CompareStats compare(const std::vector<CsvRow>& replay, const std::vector<CsvRow>& ref)
{
    CompareStats s;
    double sum_dp2 = 0, sum_dv2 = 0, sum_dang2 = 0;
    for (size_t i = 0; i < ref.size(); ++i) {
        const auto& a = replay[i].v;
        const auto& b = ref[i].v;
        const double dp = (Vector3d(a[0], a[1], a[2]) - Vector3d(b[0], b[1], b[2])).norm();
        const double dv = (Vector3d(a[3], a[4], a[5]) - Vector3d(b[3], b[4], b[5])).norm();
        // Attitude difference in degrees; q and -q are the same rotation.
        const Quaterniond qa(a[6], a[7], a[8], a[9]);
        const Quaterniond qb(b[6], b[7], b[8], b[9]);
        const double dot = std::min(1.0, std::abs(qa.normalized().dot(qb.normalized())));
        const double dang = 2.0 * std::acos(dot) * 180.0 / M_PI;

        if (dp > s.max_dp) { s.max_dp = dp; s.max_dp_row = i + 1; }
        s.max_dv = std::max(s.max_dv, dv);
        s.max_dang = std::max(s.max_dang, dang);
        sum_dp2 += dp * dp;
        sum_dv2 += dv * dv;
        sum_dang2 += dang * dang;
    }
    const double n = double(ref.size());
    s.rms_dp = std::sqrt(sum_dp2 / n);
    s.rms_dv = std::sqrt(sum_dv2 / n);
    s.rms_dang = std::sqrt(sum_dang2 / n);
    return s;
}

// Frame-invariant speed error (|v| vs |v|) for the DVL offset scan: old
// reference recordings are in the NED frame while replays now use the mission
// frame, so vector comparisons are off by a constant rotation — speed is not.
static double rmsSpeedError(const std::vector<CsvRow>& replay, const std::vector<CsvRow>& ref)
{
    double sum = 0;
    for (size_t i = 0; i < ref.size(); ++i) {
        const double d = Vector3d(replay[i].v[3], replay[i].v[4], replay[i].v[5]).norm()
                       - Vector3d(ref[i].v[3],    ref[i].v[4],    ref[i].v[5]).norm();
        sum += d * d;
    }
    return std::sqrt(sum / double(ref.size()));
}

// A run is identified by its kalman_<run>.csv (the other filenames derive from
// it). Returns the run id, or "" if the directory holds no recorded run.
// kalman_replay_*.csv files (this test's own output) are not runs.
static std::string findRunId(const fs::path& dir)
{
    for (const auto& entry : fs::directory_iterator(dir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("kalman_", 0) == 0 && name.rfind("kalman_replay_", 0) != 0
            && name.size() > 11) {
            return name.substr(7, name.size() - 7 - 4);  // strip "kalman_" and ".csv"
        }
    }
    return "";
}

int main(int argc, char* argv[])
{
    fs::path dir = (argc > 1) ? fs::path(argv[1]) : fs::path("state_estimator_outputs");

    // Runs are sorted into per-run subdirectories named by their timestamp
    // (state_estimator_outputs/<run_id>/). If `dir` holds a run directly it is
    // used as-is; otherwise pick the NEWEST run subdirectory. Pass a specific
    // run's directory as the 1st argument to replay an older run.
    std::string run_id = findRunId(dir);
    if (run_id.empty()) {
        std::vector<fs::path> run_dirs;
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (entry.is_directory() && !findRunId(entry.path()).empty())
                run_dirs.push_back(entry.path());
        }
        // Names are nanosecond timestamps: compare by length then value so
        // chronological order survives any width difference.
        std::sort(run_dirs.begin(), run_dirs.end(),
                  [](const fs::path& a, const fs::path& b) {
                      const std::string sa = a.filename().string(), sb = b.filename().string();
                      return sa.size() != sb.size() ? sa.size() < sb.size() : sa < sb;
                  });
        if (!run_dirs.empty()) {
            dir = run_dirs.back();
            run_id = findRunId(dir);
            std::cout << "Newest of " << run_dirs.size() << " runs in "
                      << dir.parent_path() << ": " << dir.filename() << std::endl;
        }
    }
    if (run_id.empty()) {
        std::cerr << "ERROR: no kalman_*.csv found in " << dir
                  << " or its subdirectories" << std::endl;
        return 1;
    }
    std::cout << "Replaying run " << run_id << " from " << dir << std::endl;

    ReplayInputs in;
    in.imu1  = loadRows(dir / ("imu1_"  + run_id + ".csv"), true);
    in.imu2  = loadRows(dir / ("imu2_"  + run_id + ".csv"), true);
    in.dvl   = loadRows(dir / ("dvl_"   + run_id + ".csv"), true);
    in.depth = loadRows(dir / ("depth_" + run_id + ".csv"), false);
    const auto ref = loadRows(dir / ("kalman_" + run_id + ".csv"), true);

    if (in.imu2.empty() || ref.empty()) {
        std::cerr << "ERROR: imu2/kalman CSVs are required and were empty" << std::endl;
        return 1;
    }
    if (ref.size() != in.imu2.size()) {
        std::cerr << "ERROR: reference kalman rows (" << ref.size()
                  << ") != imu2 rows (" << in.imu2.size()
                  << ") — the node writes one kalman row per IMU2 message" << std::endl;
        return 1;
    }
    std::cout << "Loaded: imu2=" << in.imu2.size() << " imu1=" << in.imu1.size()
              << " dvl=" << in.dvl.size() << " depth=" << in.depth.size()
              << " reference=" << ref.size() << " rows" << std::endl;

    const double t0_imu2 = in.imu2.front().t_ns * 1e-9;
    const double t_end   = in.imu2.back().t_ns * 1e-9 - t0_imu2;

    // Mission-frame anchor: usable only when the reference starts post-init
    // (new-format recordings — the node skips pre-init rows entirely).
    {
        const auto& s = ref.front().v;
        const bool pre_init = s[0] == 0 && s[1] == 0 && s[2] == 0 &&
                              s[3] == 0 && s[4] == 0 && s[5] == 0 &&
                              s[6] == 1 && s[7] == 0 && s[8] == 0 && s[9] == 0;
        if (!pre_init) {
            in.ref_q0 = Quaterniond(s[6], s[7], s[8], s[9]);
            in.have_ref_q0 = true;
        }
    }

    // Depth has no timestamps: recover the first arrival from the reference
    // output (first row whose state left the pre-init default), then spread
    // the remaining messages evenly to the end of the run.
    if (!in.depth.empty()) {
        size_t first_active_row = 0;
        for (size_t i = 0; i < ref.size(); ++i) {
            const auto& s = ref[i].v;
            const bool pre_init = s[0] == 0 && s[1] == 0 && s[2] == 0 &&
                                  s[3] == 0 && s[4] == 0 && s[5] == 0 &&
                                  s[6] == 1 && s[7] == 0 && s[8] == 0 && s[9] == 0;
            if (!pre_init) { first_active_row = i; break; }
        }
        double t_first_depth = 0.0;
        if (first_active_row > 0) {
            // Arrived between the last all-zero row and the first active one.
            t_first_depth = 0.5 * (in.imu2[first_active_row - 1].t_ns
                                   + in.imu2[first_active_row].t_ns) * 1e-9 - t0_imu2;
        }
        // Later arrivals: anchor depth rows to the reference by VALUE. R_depth
        // is tiny, so every depth update snaps the reference z to ~the measured
        // value — each z-jump in the reference is one depth row landing. Match
        // rows to jumps monotonically, then spread unmatched rows (updates too
        // small to detect) evenly between the surrounding anchors. Rows left
        // after the last anchor continue at the run's average cadence: readings
        // logged after the IMU2 stream ended (e.g. the sub surfacing with the
        // recorder still up) land past the last IMU2 event and — exactly like
        // in the live node — never affect a compared row.
        // Row 0 stays pinned to t_first_depth: it seeds frame initialization.
        std::vector<std::pair<size_t, double>> anchors{{0, t_first_depth}};
        struct Jump { double t, z; };
        std::vector<Jump> jumps;
        // A depth update shows up as z INNOVATION: z deviating from its own
        // velocity-propagated prediction. Comparing against the prediction
        // (not just the previous z) catches updates far smaller than the
        // sub's normal descent rate — a plain |dz| threshold misses updates
        // whenever the sub moves slowly and z snaps by under the threshold.
        // 8 mm sits in the gap between the two residual populations (predict
        // noise + DVL cross-corrections stay below ~6 mm; the pool runs show
        // a stable ~1 Hz jump cadence anywhere in the 6-15 mm range).
        for (size_t i = 1; i < ref.size(); ++i) {
            const double dt = (ref[i].t_ns - ref[i - 1].t_ns) * 1e-9;
            const double resid = ref[i].v[2] - (ref[i - 1].v[2] + ref[i - 1].v[5] * dt);
            if (std::abs(resid) > 0.008)
                jumps.push_back({ref[i].t_ns * 1e-9 - t0_imu2, ref[i].v[2]});
        }

        // Globally assign every jump to one depth row (row order preserved;
        // rows may be skipped — those are updates too small to see). Dynamic
        // programming: greedy matching gets trapped when z oscillates and
        // near-equal values repeat. dp[j][k] = best total |value error| for
        // the first j jumps using rows 1..k.
        const size_t J = jumps.size(), K = in.depth.size();
        if (J > 0 && K > J) {
            const double INF = 1e18;
            std::vector<std::vector<double>> dp(J + 1, std::vector<double>(K, INF));
            for (size_t k = 0; k < K; ++k) dp[0][k] = 0.0;
            for (size_t j = 1; j <= J; ++j)
                for (size_t k = j; k < K; ++k) {
                    dp[j][k] = dp[j][k - 1];  // row k unused by jump j
                    const double c = dp[j - 1][k - 1]
                                     + std::abs(in.depth[k].v[0] - jumps[j - 1].z);
                    if (c < dp[j][k]) dp[j][k] = c;
                }
            if (dp[J][K - 1] < INF) {
                std::vector<size_t> row_of(J);
                size_t k = K - 1;
                for (size_t j = J; j > 0; --j) {
                    while (k > j && dp[j][k] == dp[j][k - 1]) --k;
                    row_of[j - 1] = k--;
                }
                for (size_t j = 0; j < J; ++j)
                    if (jumps[j].t > anchors.back().second && row_of[j] > anchors.back().first)
                        anchors.push_back({row_of[j], jumps[j].t});
            }
        }

        in.depth_times.assign(in.depth.size(), 0.0);
        for (size_t a = 0; a + 1 < anchors.size(); ++a) {
            const auto [r0, ta] = anchors[a];
            const auto [r1, tb] = anchors[a + 1];
            for (size_t r = r0; r < r1; ++r)
                in.depth_times[r] = ta + (tb - ta) * double(r - r0) / double(r1 - r0);
        }
        const auto [r_last, t_last] = anchors.back();
        in.depth_times[r_last] = t_last;
        const double cadence = (r_last > 0)
            ? (t_last - t_first_depth) / double(r_last)
            : (in.depth.size() > 1 ? (t_end - t_first_depth) / double(in.depth.size() - 1)
                                   : 0.0);  // no jumps at all: even spread
        for (size_t r = r_last + 1; r < in.depth.size(); ++r)
            in.depth_times[r] = t_last + cadence * double(r - r_last);

        std::cout << "Depth timing reconstructed: first at t=" << t_first_depth
                  << " s (reference row " << first_active_row + 1 << "), "
                  << anchors.size() - 1 << " rows anchored to reference z-jumps, "
                  << "cadence " << cadence << " s" << std::endl;
    }

    // DVL clock offset: use the value from the command line, or estimate it by
    // scanning (coarse then fine) for the offset whose SPEED profile best
    // matches the reference — the DVL directly corrects velocity, and speed
    // stays comparable even when the world frame changed between recordings.
    double dvl_offset = 0.0;
    if (argc > 2) {
        dvl_offset = std::stod(argv[2]);
        std::cout << "DVL offset (from command line): " << dvl_offset << " s" << std::endl;
    } else if (!in.dvl.empty()) {
        // Silence the per-replay prints from KalmanFilter::reset() during the scan.
        std::stringstream sink;
        std::streambuf* cout_buf = std::cout.rdbuf(sink.rdbuf());

        double best_rms = -1.0;
        for (int pass = 0; pass < 2; ++pass) {
            const double half_range = (pass == 0) ? 3.0 : 0.25;
            const double step       = (pass == 0) ? 0.25 : 0.05;
            const double center     = dvl_offset;
            for (double off = center - half_range; off <= center + half_range + 1e-9; off += step) {
                const double e = rmsSpeedError(runReplay(in, off), ref);
                if (best_rms < 0 || e < best_rms) {
                    best_rms = e;
                    dvl_offset = off;
                }
            }
        }
        std::cout.rdbuf(cout_buf);
        std::cout << "DVL offset (estimated): " << dvl_offset
                  << " s (rms speed error " << best_rms << " m/s)" << std::endl;
    }

    // ---- Final replay, output CSV, and report ----
    const auto replay = runReplay(in, dvl_offset);

    // Written next to the run's input CSVs (findRunId ignores kalman_replay_*).
    const fs::path out_filename = dir / ("kalman_replay_" + run_id + ".csv");
    std::ofstream out(out_filename);
    for (const auto& row : replay) {
        out << row.t_ns;
        for (double value : row.v) out << "," << value;
        out << "\n";
    }
    out.close();
    std::cout << "Wrote " << out_filename << std::endl;

    const CompareStats s = compare(replay, ref);
    std::cout << "\n---- Replay vs recorded kalman output (" << ref.size() << " rows) ----\n"
              << "position:    rms " << s.rms_dp << " m,     max " << s.max_dp
              << " m (row " << s.max_dp_row << ")\n"
              << "velocity:    rms " << s.rms_dv << " m/s,   max " << s.max_dv << " m/s\n"
              << "orientation: rms " << s.rms_dang << " deg, max " << s.max_dang << " deg" << std::endl;

    // Regression bounds. DVL/depth arrival times are reconstructed, so the
    // replay cannot be bit-exact — these bounds catch changes to the filter
    // math itself, which shift the trajectory far more than event timing does.
    // NOTE: references recorded before the mission-frame change / DVL z-sign
    // fix are in a different world frame, so this comparison will FAIL against
    // them — record a fresh reference run to re-arm this check.
    const bool pass = s.rms_dp < 0.05 && s.rms_dv < 0.05 && s.rms_dang < 1.0;
    std::cout << (pass ? "PASS" : "FAIL")
              << " (bounds: rms position < 0.05 m, rms velocity < 0.05 m/s, rms orientation < 1 deg)"
              << std::endl;
    return pass ? 0 : 1;
}

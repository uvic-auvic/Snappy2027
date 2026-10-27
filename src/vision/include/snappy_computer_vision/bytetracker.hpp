// byte_tracker.hpp
// Header-only ByteTrack (tracking-by-detection) for the snappy vision nodes.
// Mirrors the Ultralytics BYTETracker defaults: Kalman filter on [x, y, aspect,
// height] + velocities, two-stage IoU association, unconfirmed-track handling.
// Depends only on OpenCV core (cv::Matx). Class-agnostic association.
//
// Usage: keep one ByteTracker per camera as a member, and call update() once per inferred frame
// with ALL detections above ~0.1 confidence. The tracker applies the
// high/low confidence split internally.
#pragma once
 
#include <opencv2/core.hpp>
 
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>
 
namespace snappy_tracking {
 
struct TrackInput {
  float x1, y1, x2, y2;  // box in image pixels
  float conf;
  int cls;
};
 
struct TrackOutput {
  int track_id;
  int det_index;         // index into the vector passed to update()
  float x1, y1, x2, y2;  // Kalman-smoothed box
  float conf;
  int cls;
};
 
struct ByteTrackerParams {
  float track_high_thresh = 0.25f;  // first-pass detections
  float track_low_thresh = 0.10f;   // second-pass detections (below this: ignored)
  float new_track_thresh = 0.25f;   // min conf to start a new track
  float match_thresh = 0.8f;        // max IoU-distance for first pass
  int track_buffer = 30;            // frames a lost track is kept
  int frame_rate = 30;
};
 
class ByteTracker {
 public:
  using Params = ByteTrackerParams;
 
  explicit ByteTracker(Params p = Params()) : p_(p) {
    max_time_lost_ = std::max(1, static_cast<int>(p_.frame_rate / 30.0 * p_.track_buffer));
  }
 
  void reset() {
    tracks_.clear();
    frame_ = 0;
    next_id_ = 0;
  }
 
  int frameCount() const { return frame_; }
 
  std::vector<TrackOutput> update(const std::vector<TrackInput>& dets) {
    ++frame_;
 
    // Split detections by confidence (ByteTrack's defining step).
    std::vector<int> high, low;
    for (int i = 0; i < static_cast<int>(dets.size()); ++i) {
      if (dets[i].conf >= p_.track_high_thresh) {
        high.push_back(i);
      } else if (dets[i].conf > p_.track_low_thresh) {
        low.push_back(i);
      }
    }
 
    // Predict every existing track forward one step.
    for (auto& t : tracks_) predict(t);
 
    // Confirmed (tracked or lost) vs unconfirmed (seen only once).
    std::vector<int> pool, unconfirmed;
    for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
      (tracks_[i].activated ? pool : unconfirmed).push_back(i);
    }
 
    // ---- Pass 1: confirmed tracks vs high-confidence detections ----------
    std::vector<char> high_used(high.size(), 0), pool_matched(pool.size(), 0);
    {
      const Cost cost = iouCost(pool, high, dets, /*fuse_score=*/true);
      const Assignment a = assign(cost, pool.size(), high.size(), p_.match_thresh);
      for (const auto& m : a.matches) {
        applyUpdate(tracks_[pool[m.first]], dets[high[m.second]], high[m.second]);
        pool_matched[m.first] = 1;
        high_used[m.second] = 1;
      }
    }
 
    // ---- Pass 2: leftover tracked tracks vs low-confidence detections ----
    {
      std::vector<int> r_tracked;
      for (size_t r = 0; r < pool.size(); ++r) {
        if (!pool_matched[r] && tracks_[pool[r]].state == State::Tracked) {
          r_tracked.push_back(pool[r]);
        }
      }
      const Cost cost = iouCost(r_tracked, low, dets, /*fuse_score=*/false);
      const Assignment a = assign(cost, r_tracked.size(), low.size(), 0.5);
      std::vector<char> matched(r_tracked.size(), 0);
      for (const auto& m : a.matches) {
        applyUpdate(tracks_[r_tracked[m.first]], dets[low[m.second]], low[m.second]);
        matched[m.first] = 1;
      }
      for (size_t r = 0; r < r_tracked.size(); ++r) {
        if (!matched[r]) tracks_[r_tracked[r]].state = State::Lost;
      }
    }
 
    // ---- Pass 3: unconfirmed tracks vs still-unmatched high detections ---
    std::vector<char> remove(tracks_.size(), 0);
    std::vector<int> rem_high;
    for (size_t c = 0; c < high.size(); ++c) {
      if (!high_used[c]) rem_high.push_back(high[c]);
    }
    std::vector<char> rem_high_used(rem_high.size(), 0);
    {
      const Cost cost = iouCost(unconfirmed, rem_high, dets, /*fuse_score=*/true);
      const Assignment a = assign(cost, unconfirmed.size(), rem_high.size(), 0.7);
      std::vector<char> matched(unconfirmed.size(), 0);
      for (const auto& m : a.matches) {
        applyUpdate(tracks_[unconfirmed[m.first]], dets[rem_high[m.second]], rem_high[m.second]);
        matched[m.first] = 1;
        rem_high_used[m.second] = 1;
      }
      for (size_t r = 0; r < unconfirmed.size(); ++r) {
        if (!matched[r]) remove[unconfirmed[r]] = 1;
      }
    }
 
    // ---- Birth: unmatched high detections become new tracks --------------
    std::vector<Track> fresh;
    for (size_t k = 0; k < rem_high.size(); ++k) {
      if (!rem_high_used[k] && dets[rem_high[k]].conf >= p_.new_track_thresh) {
        fresh.push_back(newTrack(dets[rem_high[k]], rem_high[k]));
      }
    }
 
    // ---- Death: lost for too long ----------------------------------------
    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (tracks_[i].state == State::Lost && frame_ - tracks_[i].frame_id > max_time_lost_) {
        remove[i] = 1;
      }
    }
    std::vector<Track> kept;
    kept.reserve(tracks_.size() + fresh.size());
    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (!remove[i]) kept.push_back(std::move(tracks_[i]));
    }
    for (auto& t : fresh) kept.push_back(std::move(t));
    tracks_ = std::move(kept);
 
    // ---- Output: confirmed tracks seen this frame ------------------------
    std::vector<TrackOutput> out;
    for (const auto& t : tracks_) {
      if (t.state == State::Tracked && t.activated) {
        const auto b = box(t.mean);
        out.push_back({t.id, t.det_index, b[0], b[1], b[2], b[3], t.score, t.cls});
      }
    }
    return out;
  }
 
 private:
  // ---- Types ---------------------------------------------------------------
  using Vec4 = cv::Matx<float, 4, 1>;
  using Vec8 = cv::Matx<float, 8, 1>;
  using Mat44 = cv::Matx<float, 4, 4>;
  using Mat48 = cv::Matx<float, 4, 8>;
  using Mat84 = cv::Matx<float, 8, 4>;
  using Mat88 = cv::Matx<float, 8, 8>;
  using Cost = std::vector<std::vector<double>>;
 
  enum class State { Tracked, Lost };
 
  struct Track {
    Vec8 mean;
    Mat88 cov;
    State state = State::Tracked;
    bool activated = false;  // false = unconfirmed (born this/last frame)
    int id = 0;
    float score = 0.f;
    int cls = 0;
    int det_index = -1;
    int frame_id = 0;  // last frame this track was matched
  };
 
  struct Assignment {
    std::vector<std::pair<int, int>> matches;  // (row, col)
  };
 
  // ---- Kalman filter (constant-velocity on [x, y, aspect, height]) ----------
  static constexpr float kWP = 1.f / 20.f;   // position noise weight
  static constexpr float kWV = 1.f / 160.f;  // velocity noise weight
 
  static Vec4 toXYAH(const TrackInput& d) {
    const float w = d.x2 - d.x1, h = d.y2 - d.y1;
    return Vec4(d.x1 + 0.5f * w, d.y1 + 0.5f * h, w / std::max(h, 1e-6f), h);
  }
 
  static std::array<float, 4> box(const Vec8& m) {
    const float h = m(3), w = m(2) * h;
    return {m(0) - 0.5f * w, m(1) - 0.5f * h, m(0) + 0.5f * w, m(1) + 0.5f * h};
  }
 
  static void kfInitiate(const Vec4& z, Vec8& mean, Mat88& cov) {
    const float h = z(3);
    mean = Vec8(z(0), z(1), z(2), z(3), 0, 0, 0, 0);
    const float sp = 2.f * kWP * h, sv = 10.f * kWV * h;
    const float s[8] = {sp, sp, 1e-2f, sp, sv, sv, 1e-5f, sv};
    cov = Mat88::zeros();
    for (int i = 0; i < 8; ++i) cov(i, i) = s[i] * s[i];
  }
 
  static void kfPredict(Vec8& mean, Mat88& cov) {
    const float h = mean(3);
    const float sp = kWP * h, sv = kWV * h;
    const float s[8] = {sp, sp, 1e-2f, sp, sv, sv, 1e-5f, sv};
    Mat88 F = Mat88::eye();
    for (int i = 0; i < 4; ++i) F(i, i + 4) = 1.f;  // dt = 1 frame
    Mat88 Q = Mat88::zeros();
    for (int i = 0; i < 8; ++i) Q(i, i) = s[i] * s[i];
    mean = F * mean;
    cov = F * cov * F.t() + Q;
  }
 
  static void kfUpdate(Vec8& mean, Mat88& cov, const Vec4& z) {
    Mat48 H = Mat48::zeros();
    for (int i = 0; i < 4; ++i) H(i, i) = 1.f;
    const float h = mean(3);
    const float sp = kWP * h;
    Mat44 R = Mat44::zeros();
    R(0, 0) = R(1, 1) = R(3, 3) = sp * sp;
    R(2, 2) = 1e-2f;  // (1e-1)^2
    const Mat44 S = H * cov * H.t() + R;
    const Mat84 K = cov * H.t() * S.inv();
    mean = mean + K * (z - H * mean);
    cov = cov - K * S * K.t();
  }
 
  // ---- Track helpers -----------------------------------------------------------
  void predict(Track& t) const {
    if (t.state != State::Tracked) t.mean(7) = 0.f;  // stop height drift while lost
    kfPredict(t.mean, t.cov);
  }
 
  void applyUpdate(Track& t, const TrackInput& d, int det_index) const {
    kfUpdate(t.mean, t.cov, toXYAH(d));
    t.state = State::Tracked;
    t.activated = true;
    t.score = d.conf;
    t.cls = d.cls;
    t.det_index = det_index;
    t.frame_id = frame_;
  }
 
  Track newTrack(const TrackInput& d, int det_index) {
    Track t;
    kfInitiate(toXYAH(d), t.mean, t.cov);
    t.id = ++next_id_;
    t.state = State::Tracked;
    t.activated = (frame_ == 1);  // later births need a second sighting to confirm
    t.score = d.conf;
    t.cls = d.cls;
    t.det_index = det_index;
    t.frame_id = frame_;
    return t;
  }
 
  // ---- Association ----------------------------------------------------------------
  static double iou(const std::array<float, 4>& a, const TrackInput& b) {
    const float ix1 = std::max(a[0], b.x1), iy1 = std::max(a[1], b.y1);
    const float ix2 = std::min(a[2], b.x2), iy2 = std::min(a[3], b.y2);
    const float inter = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
    const float ua = (a[2] - a[0]) * (a[3] - a[1]) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
    return ua > 0.f ? inter / ua : 0.0;
  }
 
  Cost iouCost(const std::vector<int>& track_idx, const std::vector<int>& det_idx,
               const std::vector<TrackInput>& dets, bool fuse_score) const {
    Cost c(track_idx.size(), std::vector<double>(det_idx.size(), 1.0));
    for (size_t r = 0; r < track_idx.size(); ++r) {
      const auto tb = box(tracks_[track_idx[r]].mean);
      for (size_t k = 0; k < det_idx.size(); ++k) {
        const TrackInput& d = dets[det_idx[k]];
        double sim = iou(tb, d);
        if (fuse_score) sim *= d.conf;
        c[r][k] = 1.0 - sim;
      }
    }
    return c;
  }
 
  // Hungarian algorithm for an n x m matrix with n <= m. Returns row -> col.
  static std::vector<int> hungarian(const Cost& a, int n, int m) {
    const double INF = 1e18;
    std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);
    for (int i = 1; i <= n; ++i) {
      p[0] = i;
      int j0 = 0;
      std::vector<double> minv(m + 1, INF);
      std::vector<char> used(m + 1, 0);
      do {
        used[j0] = 1;
        const int i0 = p[j0];
        int j1 = 0;
        double delta = INF;
        for (int j = 1; j <= m; ++j) {
          if (used[j]) continue;
          const double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
          if (minv[j] < delta) { delta = minv[j]; j1 = j; }
        }
        for (int j = 0; j <= m; ++j) {
          if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
          else { minv[j] -= delta; }
        }
        j0 = j1;
      } while (p[j0] != 0);
      do {
        const int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0);
    }
    std::vector<int> row_to_col(n, -1);
    for (int j = 1; j <= m; ++j) {
      if (p[j]) row_to_col[p[j] - 1] = j - 1;
    }
    return row_to_col;
  }
 
  // Optimal assignment, then reject pairs whose cost exceeds thresh.
  static Assignment assign(const Cost& cost, size_t rows, size_t cols, double thresh) {
    Assignment out;
    if (rows == 0 || cols == 0) return out;
    const bool transpose = rows > cols;
    const int n = static_cast<int>(transpose ? cols : rows);
    const int m = static_cast<int>(transpose ? rows : cols);
    Cost a(n, std::vector<double>(m));
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < m; ++j) {
        const double c = transpose ? cost[j][i] : cost[i][j];
        a[i][j] = (c > thresh) ? thresh + 1.0 : c;  // keep invalid pairs unattractive
      }
    }
    const std::vector<int> r2c = hungarian(a, n, m);
    for (int i = 0; i < n; ++i) {
      const int j = r2c[i];
      if (j < 0) continue;
      const int r = transpose ? j : i;
      const int c = transpose ? i : j;
      if (cost[r][c] <= thresh) out.matches.emplace_back(r, c);
    }
    return out;
  }
 
  // ---- Members ----------------------------------------------------------------------
  Params p_;
  int max_time_lost_ = 30;
  int frame_ = 0;
  int next_id_ = 0;
  std::vector<Track> tracks_;
};
 
}  // namespace snappy_tracking

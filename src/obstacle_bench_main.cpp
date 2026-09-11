// obstacle_bench: single-process bench for the obstacle-protection profile.
// Unlike the original cut-boundary profile above, this profile's claims (see
// README "Obstacle-protection profile ... honest framing") do not require an
// independently running OS process, so this bench calls ObstacleZoneModel and
// ObstacleSupervisorCore directly, the same way the sprayer profile in the
// sibling repo sprayer-section-control-hil calls its bench logic directly
// rather than over IPC, for a claim set that does not itself need one.
//
// Runs, in order, and prints a summary block for each:
//   1. the stop-distance envelope across a sweep of ground speeds
//   2. 24 seeded encounters (an obstacle at a known initial range, the
//      machine approaching at a known constant speed), each of which must
//      stop before the true range reaches 0, naming REQ-OBS-001
//   3. 200 healthy passes (obstacle always outside the warning zone), each
//      of which must produce 0 stops
//   4. detector dropout, late detection, and stale pose, each run against a
//      fault-free control on the identical encounter, pricing the lost
//      stopping margin in meters
//
// Usage: obstacle_bench [config/obstacle_zone.txt]

#include "obstacle_core.hpp"
#include "obstacle_zone.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <tuple>
#include <vector>

using namespace cbs;

namespace {

constexpr double DT_S = 0.01;             // simulation tick
constexpr double SENSOR_PERIOD_S = 0.05;  // 20 Hz detection messages
constexpr double DROPOUT_WINDOW_MS = 300.0;

enum class Fault { kNone, kDropout, kLateDetection, kStalePose };

struct ScenarioConfig {
    double obstacle_range0_m;
    double speed0_mps;
    // Optional speed step: true speed becomes speed0_mps * step_factor at
    // step_at_s (1.0 = no step). Used by the stale-pose fault so a real
    // speed change exists for a stale report to lag behind.
    double step_at_s = 1e9;
    double step_factor = 1.0;
};

struct EncounterResult {
    bool stopped = false;
    double true_range_at_stop_m = -1.0;
    std::string requirement_id;
};

double true_speed(const ScenarioConfig &sc, double t) {
    return t >= sc.step_at_s ? sc.speed0_mps * sc.step_factor : sc.speed0_mps;
}

// True range is the integral of true speed; for the piecewise-constant speed
// profile used here (at most one step), this closed form is exact.
double true_range(const ScenarioConfig &sc, double t) {
    if (t <= sc.step_at_s) return sc.obstacle_range0_m - sc.speed0_mps * t;
    double range_at_step = sc.obstacle_range0_m - sc.speed0_mps * sc.step_at_s;
    return range_at_step - sc.speed0_mps * sc.step_factor * (t - sc.step_at_s);
}

EncounterResult run_encounter(ObstacleSupervisorCore core, const ScenarioConfig &sc,
                               Fault fault, double fault_param, double dropout_cutoff_s,
                               double max_time_s = 60.0) {
    EncounterResult res;
    double next_sensor_t = 0.0;
    for (double t = 0.0; t <= max_time_s; t += DT_S) {
        double range_now = true_range(sc, t);
        if (range_now <= 0.0) break;  // collided, unresolved

        bool dropped = (fault == Fault::kDropout && t >= dropout_cutoff_s);
        if (t + 1e-9 >= next_sensor_t && !dropped) {
            double report_t_range = (fault == Fault::kLateDetection) ? std::max(0.0, t - fault_param) : t;
            double report_t_speed = (fault == Fault::kStalePose) ? std::max(0.0, t - fault_param) : t;
            double rep_range = true_range(sc, report_t_range);
            double rep_speed = true_speed(sc, report_t_speed);

            ObstacleDecision d = core.decide_detection(rep_range, rep_speed, t * 1000.0);
            if (d.kind == ObstacleDecisionKind::kStopHazard) {
                res.stopped = true;
                res.true_range_at_stop_m = range_now;
                res.requirement_id = d.requirement_id;
                return res;
            }
            next_sensor_t += SENSOR_PERIOD_S;
        }

        ObstacleDecision dd = core.poll_dropout(t * 1000.0);
        if (dd.kind == ObstacleDecisionKind::kStopDropout) {
            res.stopped = true;
            res.true_range_at_stop_m = range_now;
            res.requirement_id = dd.requirement_id;
            return res;
        }
    }
    return res;
}

int g_pass = 0, g_fail = 0;

}  // namespace

int main(int argc, char **argv) {
    std::string config_path = argc > 1 ? argv[1] : "config/obstacle_zone.txt";
    ObstacleZoneModel zone = ObstacleZoneModel::load_from_file(config_path);

    std::printf("=== Stop-distance envelope across ground speeds ===\n");
    std::printf("speed_mps,stopping_distance_m,hazard_boundary_m,warning_boundary_m\n");
    for (double v = 0.25; v <= 3.01; v += 0.25) {
        std::printf("%.2f,%.4f,%.4f,%.4f\n", v, zone.stopping_distance_m(v),
                     zone.hazard_boundary_m(v), zone.warning_boundary_m(v));
    }

    std::printf("\n=== 24 seeded encounters ===\n");
    int encounters_held = 0;
    const std::vector<double> speeds = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    const std::vector<double> range_margins_m = {2.0, 5.0, 10.0, 20.0};  // range0 = hazard_boundary(v) + margin
    int idx = 0;
    for (double v : speeds) {
        for (double extra : range_margins_m) {
            ++idx;
            ScenarioConfig sc{zone.hazard_boundary_m(v) + extra, v};
            ObstacleSupervisorCore core(zone, DROPOUT_WINDOW_MS);
            EncounterResult r = run_encounter(core, sc, Fault::kNone, 0.0, 1e9);
            bool ok = r.stopped && r.true_range_at_stop_m > 0.0 && r.requirement_id == "REQ-OBS-001";
            if (ok) ++encounters_held;
            std::printf("encounter %2d: speed=%.2f range0=%.3f -> stopped=%d true_range_at_stop=%.4f req=%s %s\n",
                        idx, v, sc.obstacle_range0_m, r.stopped, r.true_range_at_stop_m,
                        r.requirement_id.c_str(), ok ? "HELD" : "MISS");
        }
    }
    std::printf("encounters_held: %d / %d\n", encounters_held, idx);

    std::printf("\n=== 200 healthy passes ===\n");
    int false_stops = 0;
    for (int i = 0; i < 200; ++i) {
        double v = 0.4 + 0.013 * static_cast<double>(i % 200);  // 0.4..3.0 m/s spread
        // Obstacle always well outside the warning zone for the whole approach
        // window: place it at warning_boundary(v) + a generous fixed clearance,
        // and stop the simulated approach before it would ever close that gap.
        double clearance_m = 15.0;
        ScenarioConfig sc{zone.warning_boundary_m(v) + clearance_m, v};
        ObstacleSupervisorCore core(zone, DROPOUT_WINDOW_MS);
        // Only simulate long enough to travel a fraction of the clearance, so
        // the obstacle genuinely never approaches either zone in this pass.
        EncounterResult r = run_encounter(core, sc, Fault::kNone, 0.0, 1e9, /*max_time_s=*/5.0);
        if (r.stopped) ++false_stops;
    }
    std::printf("false_stops: %d / 200\n", false_stops);

    std::printf("\n=== Fault injection: lost stopping margin ===\n");
    struct FaultSpec { const char *name; Fault kind; double param; };
    std::vector<FaultSpec> faults = {
        {"detector_dropout", Fault::kDropout, 0.0},
        {"late_detection", Fault::kLateDetection, 0.40},
        {"stale_pose", Fault::kStalePose, 0.60},
    };
    for (const FaultSpec &fs : faults) {
        double total_lost_m = 0.0;
        int reps = 8;
        for (int i = 0; i < reps; ++i) {
            double v = 1.0 + 0.3 * static_cast<double>(i);
            ScenarioConfig sc{zone.hazard_boundary_m(v) + 8.0, v};
            if (fs.kind == Fault::kStalePose) { sc.step_at_s = 2.0; sc.step_factor = 1.6; }

            ObstacleSupervisorCore control_core(zone, DROPOUT_WINDOW_MS);
            EncounterResult control = run_encounter(control_core, sc, Fault::kNone, 0.0, 1e9);

            ObstacleSupervisorCore fault_core(zone, DROPOUT_WINDOW_MS);
            double dropout_cutoff_s = fs.kind == Fault::kDropout ? 1.0 : 1e9;
            EncounterResult faulty = run_encounter(fault_core, sc, fs.kind, fs.param, dropout_cutoff_s);

            double lost_m = control.true_range_at_stop_m - faulty.true_range_at_stop_m;
            total_lost_m += lost_m;
            std::printf("%s rep=%d speed=%.2f control_stop_range=%.4f faulty_stop_range=%.4f lost_margin_m=%.4f\n",
                        fs.name, i, v, control.true_range_at_stop_m, faulty.true_range_at_stop_m, lost_m);
        }
        std::printf("%s: mean_lost_margin_m=%.4f (n=%d)\n", fs.name, total_lost_m / reps, reps);
    }

    std::printf("\n=== SUMMARY ===\n");
    std::printf("encounters_held=%d\n", encounters_held);
    std::printf("encounters_total=%d\n", idx);
    std::printf("false_stops=%d\n", false_stops);
    std::printf("healthy_passes_total=200\n");

    (void)g_pass; (void)g_fail;
    // Non-zero exit on any encounter miss or any false stop, so CI actually
    // gates on this bench's own pass/fail criteria, not just "it ran".
    bool ok = (encounters_held == idx) && (false_stops == 0);
    return ok ? 0 : 1;
}

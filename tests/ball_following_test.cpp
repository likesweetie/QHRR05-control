#include <mujoco/mujoco.h>
#include "motion_controllers/ball_following_controller.hpp"
#include "interfaces/input/joystick.hpp"
#include "ipc/shm_utils.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <sys/stat.h>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

double Distance(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    double result = 0;

    for (int i = 0; i < 3; ++i)
        result += std::pow(a[i] - b[i], 2);

    return std::sqrt(result);
}

int main(int argc, char** argv) {
    std::string directory;

    try {
        Require(argc == 4, "expected control, model, input config");
        qhrr05::BallFollowingController follower(argv[1]);
        Require(follower.workspace_points() > 50, "empty/trivial safe FK workspace");
        const auto surface = follower.WorkspaceSurface();
        Require(surface.size() > 12, "safe workspace has no displayable boundary");

        for (const auto& triangle : surface)
            for (double coordinate : triangle)
                Require(std::isfinite(coordinate), "workspace surface contains invalid coordinates");
        const auto config = YAML::LoadFile(argv[1]);
        const auto home_values = config["homing"]["position_rad"].as<std::vector<double>>();
        const std::array<double, 3> home{home_values[0], home_values[1], home_values[2]};
        const std::array<double, 3> zero{};
        std::array<double, 3> position_reference{}, torque{};

        follower.Start(home);
        const auto entry = follower.ball_position();
        Require(follower.joint_reference() == home, "mode entry jumped the reference");

        // Trigger speed changes affect only the ball; L2 wins when both are held.
        const std::array<double, 3> direction{1, 0, 0};
        const double dt = .002;
        double speeds[4]{};
        const std::array<double, 3> measured_velocity{.5, -.4, .3};

        for (int mode = 0; mode < 4; ++mode) {
            follower.SetBallPosition(entry);
            follower.Compute(dt, direction, mode == 1 || mode == 3, mode >= 2,
                true, home, measured_velocity, zero, position_reference, torque);
            speeds[mode] = Distance(follower.ball_position(), entry) / dt;
            for (int i = 0; i < 3; ++i) {
                const double limit = config["limits"]["torque_nm"][i].as<double>();
                const double expected = 0;
                Require(position_reference == follower.joint_reference(),
                    "ball position reference was replaced by measured position or a torque offset");
                Require(std::abs(torque[i] - std::clamp(expected, -limit, limit)) < 1e-12,
                    "ball torque added joint position/velocity feedback");
            }
        }

        Require(std::abs(speeds[0] - .10) < 1e-9 && std::abs(speeds[1] - .03) < 1e-9 &&
                    std::abs(speeds[2] - .30) < 1e-9 && std::abs(speeds[3] - .03) < 1e-9,
            "R2/L2/normal speed mapping is wrong");
        Require(follower.foot_speed() == .08, "triggers changed foot speed");

        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        const int site = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");
        double greatest_step = 0;

        // Two nearby targets must resolve to distinct interior FK interpolants,
        // rather than waiting for the nearest grid vertex to change.
        std::array<double, 3> interpolated_goals[2];
        follower.Start(home);
        for (int target = 0; target < 2; ++target) {
            auto position = entry;
            position[0] += .001 * (target + 1);
            follower.SetBallPosition(position);
            for (int i = 0; i < 6000; ++i)
                follower.Compute(.001, zero, 0, 0, false, home, zero, zero, position_reference, torque);
            Require(!follower.workspace_limited(), "interior FK cell was not available for interpolation");
            interpolated_goals[target] = follower.nearest_position();
            Require(Distance(follower.toe_reference(), interpolated_goals[target]) < 1e-5,
                "reference did not settle to the interpolated FK target");
            Require(Distance(interpolated_goals[target], position) < .006,
                "coarse FK interpolation error unexpectedly exceeded 6 mm");
        }
        const double mapped_step = Distance(interpolated_goals[0], interpolated_goals[1]);
        Require(mapped_step > .0005 && mapped_step < .0015,
            "sub-grid target change was lost or became a grid-size jump");

        follower.Start(home);
        auto previous = follower.toe_reference();
        std::array<double, 3> previous_velocity{}, previous_acceleration{};
        double maximum_acceleration = 0, maximum_jerk = 0, pause = 0, longest_pause = 0;
        std::vector<double> compute_us;
        for (int i = 0; i < 9000; ++i) {
            const double input = i >= 1000 && i < 3000 ? .2 : i >= 3000 && i < 5000 ? -.2 : 0;
            const auto begin = std::chrono::steady_clock::now();
            follower.Compute(.001, {input, 0, 0}, 0, 0, true,
                home, measured_velocity, zero, position_reference, torque);
            compute_us.push_back(std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - begin).count());
            const auto position = follower.toe_reference();
            std::array<double, 3> velocity{}, acceleration{};
            double speed_squared = 0, acceleration_squared = 0, jerk_squared = 0;
            for (int axis = 0; axis < 3; ++axis) {
                velocity[axis] = (position[axis] - previous[axis]) / .001;
                acceleration[axis] = (velocity[axis] - previous_velocity[axis]) / .001;
                speed_squared += velocity[axis] * velocity[axis];
                acceleration_squared += acceleration[axis] * acceleration[axis];
                jerk_squared += std::pow((acceleration[axis] - previous_acceleration[axis]) / .001, 2);
                Require(torque[axis] == 0, "FK interpolation added a joint PD torque");
            }
            maximum_acceleration = std::max(maximum_acceleration, std::sqrt(acceleration_squared));
            maximum_jerk = std::max(maximum_jerk, std::sqrt(jerk_squared));
            pause = input != 0 && speed_squared < 1e-10 ? pause + .001 : 0;
            longest_pause = std::max(longest_pause, pause);
            Require(!follower.workspace_limited(), "interior stick motion left the FK interpolation cells");
            Require(std::sqrt(speed_squared) <= follower.foot_speed() * 1.02,
                "smoothed FK path exceeded foot speed");
            previous = position;
            previous_velocity = velocity;
            previous_acceleration = acceleration;
        }
        Require(longest_pause < .015, "FK interpolant still pauses between grid vertices");
        Require(maximum_acceleration < 1 && maximum_jerk < 20,
            "stick start, reversal or release made a discontinuous FK reference");
        Require(Distance(follower.toe_reference(), follower.nearest_position()) < 1e-5,
            "released stick did not settle at the FK interpolant");
        std::sort(compute_us.begin(), compute_us.end());
        std::cout << "PASS: FK cell interpolation, 1 mm targets, start/reversal/stop; acceleration="
                  << maximum_acceleration << " m/s2 jerk=" << maximum_jerk
                  << " m/s3 longest pause=" << longest_pause << " s compute p99="
                  << compute_us[compute_us.size() * 99 / 100] << " us\n";
        follower.Start(home);

        // Ball jumps to six unreachable targets; the reference must remain Lipschitz,
        // regular and within joint limits, then hold the selected nearest safe FK point.
        for (const auto& target : {std::array<double, 3>{2, .14, .2}, {-2, .14, .2},
                 {.2, 2, .2}, {.2, -2, .2}, {.2, .14, 2}, {.2, .14, -2}}) {
            const auto before_jump = follower.joint_reference();
            follower.SetBallPosition(target);
            Require(follower.joint_reference() == before_jump, "target jump changed joint reference");

            for (int step = 0; step < 12000; ++step) {
                const auto previous = follower.toe_reference();
                follower.Compute(dt, zero, 0, 0, false, home, zero, zero, position_reference, torque);
                greatest_step = std::max(greatest_step, Distance(previous, follower.toe_reference()));
                Require(Distance(previous, follower.toe_reference()) <= follower.foot_speed() * dt * 1.02 + 1e-8,
                    "reference position jumped or exceeded the fixed FK travel speed");

                if (step % 50 == 0) {
                    const auto q = follower.joint_reference();

                    for (int i = 0; i < 3; ++i) {
                        Require(q[i] >= model->jnt_range[2 * i] && q[i] <= model->jnt_range[2 * i + 1],
                            "joint reference left the physical range");
                        data->qpos[i] = q[i];
                    }

                    mj_forward(model.get(), data.get());
                    double j[9], matrix[9]{};
                    mj_jacSite(model.get(), data.get(), j, nullptr, site);

                    for (int i = 0; i < 3; ++i) {
                        for (int k = 0; k < 3; ++k)
                            for (int n = 0; n < 3; ++n)
                                matrix[3 * i + k] += j[3 * i + n] * j[3 * k + n];
                    }

                    double eigenvalues[3], eigenvectors[9], quaternion[4];
                    mju_eig3(eigenvalues, eigenvectors, quaternion, matrix);
                    const double minimum = *std::min_element(eigenvalues, eigenvalues + 3);
                    const double maximum = *std::max_element(eigenvalues, eigenvalues + 3);
                    const double limit = config["workspace"]["maximum_condition_number"].as<double>();
                    Require(minimum > 0 && maximum <= limit * limit * minimum * (1 + 1e-8),
                        "FK reference path exceeded the configured Jacobian condition limit");
                }
            }

            Require(follower.workspace_limited() && !follower.entering_workspace(),
                "outside workspace did not remain projected");
            if (Distance(follower.toe_reference(), follower.nearest_position()) >= 1e-8)
                std::cerr << "FK endpoint residual=" << Distance(follower.toe_reference(), follower.nearest_position())
                          << " target=" << target[0] << ',' << target[1] << ',' << target[2]
                          << " velocity=" << follower.joint_velocity_reference()[0] << ','
                          << follower.joint_velocity_reference()[1] << ','
                          << follower.joint_velocity_reference()[2] << '\n';
            Require(Distance(follower.toe_reference(), follower.nearest_position()) < 1e-8,
                "foot did not hold its nearest reachable FK endpoint");
            Require(follower.ball_position() == target, "unreachable ball target was silently clamped");
        }

        follower.Start(home);
        Require(Distance(follower.ball_position(), entry) < 1e-12 &&
                    follower.joint_reference() == home,
            "mode reentry did not reset ball/current reference");
        std::cout << "PASS: FK workspace " << follower.workspace_points()
                  << " samples, 6 outside targets, regular paths, reference max step="
                  << greatest_step << " m, jump continuity, reentry, trigger speeds\n";

        char temporary[] = "/tmp/qhrr05-ball-input-XXXXXX";
        Require(::mkdtemp(temporary), "temporary directory failed");
        directory = temporary;

        // The same coarse FK construction must respond to the configurable IK condition limit.
        auto workspace_config = YAML::Clone(config);
        workspace_config["model_path"] = std::filesystem::absolute(argv[2]).string();
        const auto workspace_path = directory + "/workspace.yaml";
        std::size_t counts[2];

        for (int i = 0; i < 2; ++i) {
            workspace_config["workspace"]["maximum_condition_number"] = i ? 20 : 5;
            std::ofstream(workspace_path) << workspace_config;
            qhrr05::BallFollowingController workspace(workspace_path);
            counts[i] = workspace.workspace_points();
            Require(!workspace.WorkspaceSurface().empty(), "condition-filtered workspace has no rough mesh");
        }

        Require(counts[0] < follower.workspace_points() && follower.workspace_points() < counts[1],
            "changing the condition threshold did not change the sampled workspace");

        for (int invalid = 0; invalid < 4; ++invalid) {
            auto settings = YAML::Clone(workspace_config);

            if (invalid == 0)
                settings["workspace"]["maximum_condition_number"] = .5;
            else if (invalid == 1)
                settings["workspace"].remove("maximum_condition_number");
            else if (invalid == 2)
                settings["ball_following"]["minimum_singular_value_m"] = .012;
            else
                settings["ball_following"]["smoothing_sec"] = 0;

            std::ofstream(workspace_path) << settings;
            bool rejected = false;

            try {
                qhrr05::BallFollowingController workspace(workspace_path);
            } catch (const std::exception&) {
                rejected = true;
            }

            Require(rejected, "invalid or obsolete workspace condition setting was silently accepted");
        }

        std::cout << "PASS: condition limits 5/10/20 retain " << counts[0] << "/"
                  << follower.workspace_points() << "/" << counts[1] << " FK samples; invalid settings rejected\n";

        // Real Linux js_event ingestion over an explicit virtual FIFO, never a host joystick.
        const auto fifo = directory + "/joystick";
        int writer = -1;
        auto input_config = YAML::LoadFile(argv[3]);
        input_config["device"] = fifo;
        const auto input_path = directory + "/input.yaml";
        std::ofstream(input_path) << input_config;
        {
            shm_utils::Mapping owner("/qhrr05-ball-input-" + std::to_string(getpid()), true);
            Joystick joystick(input_path.c_str(), owner.get());
            joystick.initialize();
            joystick.start();
            const auto snapshot = [&] {
                shm_utils::Lock lock(*owner.get());
                return owner.get()->control[owner.get()->control_sequence % kShmBufferCount];
            };
            const auto initial = snapshot();
            Require(joystick.isRunning() && !initial.joystick_connected &&
                    initial.joystick_axes == zero && initial.joystick_l2 == 0 && initial.joystick_r2 == 0,
                "missing joystick stopped the reader or published input");
            const auto send = [&](int axis, int value) {
                js_event event{};
                event.type = JS_EVENT_AXIS | JS_EVENT_INIT;
                event.number = axis;
                event.value = value;
                Require(::write(writer, &event, sizeof(event)) == sizeof(event), "event write failed");
            };
            const auto until = [&](auto predicate) {
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

                while (!predicate(snapshot())) {
                    Require(std::chrono::steady_clock::now() < deadline, "joystick event not published");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            };

            auto last_attempt = initial;
            for (int attempt = 0; attempt < 2; ++attempt) {
                until([&](const ControlData& c) { return c.joystick_sequence > last_attempt.joystick_sequence; });
                const auto next = snapshot();
                const double seconds = (next.joystick_published_ns - last_attempt.joystick_published_ns) / 1e9;
                Require(seconds >= .90 && seconds < 1.8 && joystick.isRunning() && !next.joystick_connected,
                    "missing joystick was not retried once per second");
                last_attempt = next;
            }
            Require(::mkfifo(fifo.c_str(), 0600) == 0, "joystick FIFO failed");
            writer = ::open(fifo.c_str(), O_RDWR | O_NONBLOCK);
            Require(writer >= 0, "virtual joystick open failed");
            until([](const ControlData& c) { return c.joystick_connected; });
            Require(snapshot().joystick_l2 == 0 && snapshot().joystick_r2 == 0,
                "newly connected trigger appeared pressed");
            send(1, -32767);
            send(5, 32767);
            until([](const ControlData& c) { return c.joystick_axes[0] == 1 && c.joystick_r2 == 1; });
            send(2, 32767);
            until([](const ControlData& c) { return c.joystick_l2 == 1 && c.joystick_r2 == 1; });
            js_event button{};
            button.type = JS_EVENT_BUTTON;
            button.number = 0;
            button.value = 1;
            Require(::write(writer, &button, sizeof(button)) == sizeof(button), "button event write failed");
            until([](const ControlData& c) { return c.joystick_buttons[0]; });
            ::close(writer);
            writer = -1;
            Require(::unlink(fifo.c_str()) == 0, "virtual disconnect unlink failed");
            until([](const ControlData& c) { return !c.joystick_connected; });
            const auto disconnected = snapshot();
            Require(!disconnected.joystick_connected && disconnected.joystick_axes == zero &&
                        disconnected.joystick_r2 == 0 && disconnected.joystick_l2 == 0 &&
                        !disconnected.joystick_buttons[0] && joystick.isRunning(),
                "disconnect left stale joystick commands");
            Require(::mkfifo(fifo.c_str(), 0600) == 0, "reconnect FIFO failed");
            writer = ::open(fifo.c_str(), O_RDWR | O_NONBLOCK);
            Require(writer >= 0, "reconnect writer failed");
            until([](const ControlData& c) { return c.joystick_connected; });
            const auto reconnected = snapshot();
            const double retry_seconds = (reconnected.joystick_published_ns - disconnected.joystick_published_ns) / 1e9;
            Require(retry_seconds >= .9 && retry_seconds < 1.8 && reconnected.joystick_axes == zero &&
                    reconnected.joystick_l2 == 0 && reconnected.joystick_r2 == 0 &&
                    !reconnected.joystick_buttons[0], "reconnect reused old input or bypassed retry interval");
            send(1, 32767);
            until([](const ControlData& c) { return c.joystick_axes[0] == -1; });
            ::close(writer);
            writer = -1;
            Require(::unlink(fifo.c_str()) == 0, "second virtual disconnect failed");
            until([](const ControlData& c) { return !c.joystick_connected; });
            const auto stop_started = std::chrono::steady_clock::now();
            joystick.stop();
            joystick.join();
            Require(std::chrono::steady_clock::now() - stop_started < std::chrono::milliseconds(200),
                "stopping waited for the reconnect timer");
        }

        if (writer >= 0) ::close(writer);
        std::filesystem::remove_all(directory);
        std::cout << "PASS: virtual joystick 1 s retry, hot connect, axes/buttons/triggers reset, reconnect, prompt stop\n";
        return 0;
    } catch (const std::exception& error) {
        if (!directory.empty())
            std::filesystem::remove_all(directory);

        std::cerr << error.what() << '\n';
        return 1;
    }
}

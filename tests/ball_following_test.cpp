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
        std::array<double, 3> torque{};
        follower.Start(home);
        const auto entry = follower.ball_position();
        Require(follower.joint_reference() == home, "mode entry jumped the reference");

        // Trigger speed changes affect only the ball; L2 wins when both are held.
        const std::array<double, 3> direction{1, 0, 0};
        const double dt = .002;
        double speeds[4]{};

        for (int mode = 0; mode < 4; ++mode) {
            follower.SetBallPosition(entry);
            follower.Compute(dt, direction, mode == 1 || mode == 3, mode >= 2,
                true, home, zero, zero, torque);
            speeds[mode] = Distance(follower.ball_position(), entry) / dt;
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

        // Ball jumps to six unreachable targets; the reference must remain Lipschitz,
        // regular and within joint limits, then hold the selected nearest safe FK point.
        for (const auto& target : {std::array<double, 3>{2, .14, .2}, {-2, .14, .2},
                 {.2, 2, .2}, {.2, -2, .2}, {.2, .14, 2}, {.2, .14, -2}}) {
            const auto before_jump = follower.joint_reference();
            follower.SetBallPosition(target);
            Require(follower.joint_reference() == before_jump, "target jump changed joint reference");

            for (int step = 0; step < 12000; ++step) {
                const auto previous = follower.toe_reference();
                follower.Compute(dt, zero, 0, 0, false, home, zero, zero, torque);
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

        for (int invalid = 0; invalid < 3; ++invalid) {
            auto settings = YAML::Clone(workspace_config);

            if (invalid == 0)
                settings["workspace"]["maximum_condition_number"] = .5;
            else if (invalid == 1)
                settings["workspace"].remove("maximum_condition_number");
            else
                settings["ball_following"]["minimum_singular_value_m"] = .012;

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
        Require(::mkfifo(fifo.c_str(), 0600) == 0, "joystick FIFO failed");
        const int writer = ::open(fifo.c_str(), O_RDWR | O_NONBLOCK);
        Require(writer >= 0, "virtual joystick open failed");
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
            Require(initial.joystick_connected && initial.joystick_l2 == 0 && initial.joystick_r2 == 0,
                "uninitialized trigger appeared pressed");
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

            send(1, -32767);
            send(5, 32767);
            until([](const ControlData& c) { return c.joystick_axes[0] == 1 && c.joystick_r2 == 1; });
            send(2, 32767);
            until([](const ControlData& c) { return c.joystick_l2 == 1 && c.joystick_r2 == 1; });
            joystick.stop();
            joystick.join();
            const auto disconnected = snapshot();
            Require(!disconnected.joystick_connected && disconnected.joystick_axes == zero &&
                        disconnected.joystick_r2 == 0 && disconnected.joystick_l2 == 0,
                "disconnect left stale joystick commands");
        }

        ::close(writer);
        std::filesystem::remove_all(directory);
        std::cout << "PASS: virtual Linux joystick init, axes, R2/L2, disconnect\n";
        return 0;
    } catch (const std::exception& error) {
        if (!directory.empty())
            std::filesystem::remove_all(directory);

        std::cerr << error.what() << '\n';
        return 1;
    }
}

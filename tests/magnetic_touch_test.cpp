#include "motion_controllers/magnetic_touch_controller.hpp"
#include "mujoco_simulate/simulation.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <unistd.h>

void Require(bool v, const char* message) {
    if (!v)
        throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    try {
        Require(argc == 3, "expected config and scene");
        qhrr05::ReactionForceObserver observer(argv[1]);
        qhrr05::MotionFeedback feedback;
        feedback.measurement_dt = .001;
        feedback.jacobian = {.18, .02, -.01, .04, .25, .03, .01, -.02, .22};
        feedback.mass = {.1, 0, 0, 0, .2, 0, 0, 0, .3};
        feedback.bias_torque = {1, 2, 3};
        feedback.acceleration = {.3, -.2, .1};
        feedback.passive_torque = {-.2, .1, -.3};
        const double expected[] = {2, -1, 3};

        for (int i = 0; i < 3; ++i) {
            feedback.motor_torque[i] = feedback.mass[3 * i + i] * feedback.acceleration[i] +
                                       feedback.bias_torque[i] - feedback.passive_torque[i];

            for (int j = 0; j < 3; ++j)
                feedback.motor_torque[i] -= feedback.jacobian[3 * j + i] * expected[j];
        }

        for (int i = 1; i <= 300; ++i) {
            feedback.measurement_sequence = i;
            observer.Update(feedback);
        }

        Require(observer.valid(), "RFOB rejected a full-rank Jacobian");

        for (int i = 0; i < 3; ++i)
            Require(std::abs(observer.force()[i] - expected[i]) < .001,
                "RFOB residual force/sign is wrong");

        const auto previous = observer.force();
        feedback.motor_torque.fill(100);
        observer.Update(feedback);
        Require(observer.force() == previous, "RFOB processed a repeated measurement");
        feedback.jacobian.fill(0);
        observer.Update(feedback);
        Require(!observer.valid(), "RFOB accepted a singular pose");

        qhrr05::MagneticTouchController magnetic(argv[1]);
        feedback = {};
        feedback.measurement_dt = .001;
        feedback.jacobian = {.2, 0, 0, 0, .2, 0, 0, 0, .2};
        feedback.mass = {.1, 0, 0, 0, .1, 0, 0, 0, .1};
        feedback.position = {0, .9, -1.8};
        feedback.gravity_torque = feedback.bias_torque = {1, 2, -.5};
        feedback.toe_position = {.2, .14, .18};
        feedback.joint_min = {-1, -1, -3};
        feedback.joint_max = {1, 3, -.5};
        std::array<double, 3> torque, reference;
        magnetic.Start(0, feedback);
        double release_position = 0, release_start = -1;

        for (int i = 1; i <= 700; ++i) {
            feedback.measurement_sequence = i;
            const double external = i > 200 && i < 550 ? -2.0 : 0.0;
            feedback.motor_torque = feedback.gravity_torque;
            feedback.motor_torque[0] -= .2 * external;
            magnetic.Compute(i * .001, feedback, 3, torque, reference);
            const auto& telemetry = magnetic.telemetry();

            if (telemetry.phase == MagneticPhase::Releasing) {
                if (release_start < 0) {
                    release_start = i * .001;
                    release_position = telemetry.displacement;
                }

                Require(telemetry.displacement == release_position,
                    "lost contact continued position integration");
            }

            if (telemetry.phase == MagneticPhase::Free &&
                telemetry.reason == MagneticReason::ContactLost)
                Require(i * .001 - release_start >= .099, "contact released before debounce delay");

            Require(telemetry.force_command >= 0 && telemetry.force_command <= 5,
                "force command limit failed");
            Require(std::abs(telemetry.displacement) <= .015, "admittance distance limit failed");

            if (i == 190)
                Require(telemetry.phase == MagneticPhase::Free, "false free-space detection");

            if (i == 500) {
                Require(telemetry.phase == MagneticPhase::Following, "contact was not detected");
                Require(telemetry.normal[0] > .99, "push direction sign is wrong");
                Require(torque[0] > feedback.gravity_torque[0],
                    "contact force did not affect torque target");
            }

            if (i == 690)
                Require(telemetry.phase == MagneticPhase::Free &&
                            telemetry.reason == MagneticReason::ContactLost,
                    "contact release failed");
        }

        feedback.toe_position[0] = 1;
        magnetic.Compute(.701, feedback, 3, torque, reference);
        Require(magnetic.telemetry().reason == MagneticReason::Workspace,
            "workspace exit was not reported");

        feedback.toe_position = {.2, .14, .18};
        qhrr05::MagneticTouchController bounded(argv[1]);
        bounded.Start(1, feedback);
        feedback.motor_torque = feedback.gravity_torque;
        feedback.motor_torque[0] += .4;

        for (int i = 1; i <= 300; ++i) {
            feedback.measurement_sequence = 800 + i;
            bounded.Compute(1 + i * .001, feedback, 3, torque, reference);
        }

        Require(bounded.telemetry().phase == MagneticPhase::Following,
            "bounded trial did not enter following");
        feedback.toe_position[0] += .02;
        bounded.Compute(1.301, feedback, 3, torque, reference);
        Require(bounded.telemetry().phase == MagneticPhase::Free &&
                    bounded.telemetry().reason == MagneticReason::StrokeLimit,
            "actual following distance limit did not release");
        feedback.toe_position = {.2, .14, .18};
        feedback.position[0] = .99;
        bounded.Compute(1.302, feedback, 3, torque, reference);
        Require(bounded.telemetry().reason == MagneticReason::JointLimit,
            "joint limit was not reported");
        feedback.position[0] = 0;
        feedback.jacobian.fill(0);
        bounded.Compute(1.303, feedback, 3, torque, reference);
        Require(bounded.telemetry().reason == MagneticReason::ObserverInvalid,
            "invalid observer did not disable following");
        feedback.gravity_torque[0] = 1000;
        bounded.Compute(1.304, feedback, 3, torque, reference);
        Require(bounded.telemetry().torque_saturated && torque[0] == 23.7,
            "magnetic torque saturation failed");
        bool target_rejected = false;

        try {
            bounded.Compute(1.305, feedback, 6, torque, reference);
        } catch (const std::exception&) {
            target_rejected = true;
        }

        Require(target_rejected, "out-of-range target force was accepted");

        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(model != nullptr, error);
        const int foot = mj_name2id(model.get(), mjOBJ_GEOM, "FL");
        Require(model->geom_condim[foot] == 6, "foot condim changed");
        Require(model->actuator_gainprm[0] == 1 && model->actuator_biasprm[1] == 0,
            "torque motor changed to servo");
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> gravity(
            mj_makeData(model.get()), mj_deleteData);
        const auto config = YAML::LoadFile(argv[1]);
        const auto home = qhrr05::ReadPosition(config["homing"]["position_rad"], model.get());
        qhrr05::ResetSimulation(home, model.get(), data.get());
        const int object = mj_name2id(model.get(), mjOBJ_BODY, "touch_object");
        const int mocap = model->body_mocapid[object];
        const int toe = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");
        const double toe_x = data->site_xpos[3 * toe], toe_y = data->site_xpos[3 * toe + 1],
                     toe_z = data->site_xpos[3 * toe + 2];
        data->mocap_pos[3 * mocap + 1] = toe_y;
        data->mocap_pos[3 * mocap + 2] = toe_z;
        shm_utils::Mapping mapping("/qhrr05-magnetic-test-" + std::to_string(getpid()), true);
        auto& shm = *mapping.get();
        qhrr05::MagneticTouchController controller(argv[1]);
        controller.Start(0, feedback);
        std::array<double, 3> acceleration{};
        double sum_gt = 0, sum_est = 0, sum_squared = 0, peak = 0;
        int count = 0, following = 0;
        std::ofstream csv("/tmp/qhrr05-magnetic-physics.csv");
        csv << "t,phase,reason,est,gt,cmd,displacement,toe_x,object_x,moment\n";

        for (int step = 0; step < 4000; ++step) {
            const double time = data->time;
            // Approach, press, hold, move slightly while pressed, then withdraw.
            double center = toe_x + .09;

            if (time >= .3 && time < .9)
                center -= (time - .3) * .1;
            else if (time >= .9 && time < 2.8)
                center = toe_x + .03 + .002 * std::sin((time - .9) * 2);
            else if (time >= 2.8)
                center = toe_x + .09;
            data->mocap_pos[3 * mocap] = center;
            qhrr05::UpdateControlIO(shm, model.get(), data.get(), gravity.get(), reference, step);
            qhrr05::MotionFeedback state;
            static_cast<ModelData&>(state) = shm.model[shm.model_sequence % 2];
            const auto motors = shm.motors[shm.motor_sequence % 2];
            state.acceleration = acceleration;
            state.measurement_sequence = step;
            state.measurement_dt = model->opt.timestep;

            for (int i = 0; i < 3; ++i) {
                state.position[i] = motors.position_offset[i] + motors.feedback[i].position;
                state.velocity[i] = motors.feedback[i].velocity;
                state.motor_torque[i] = motors.feedback[i].current * motors.profile.kt_output;
            }
            controller.Compute(time, state, 3, torque, reference);
            {
                shm_utils::Lock lock(shm);
                auto& motors = shm_utils::Next(shm.motors, shm.motor_sequence);

                for (int i = 0; i < 3; ++i)
                    motors.debug_torque[i] = torque[i];

                ++shm.motor_sequence;
            }

            const std::array<double, 3> velocity{data->qvel[0], data->qvel[1], data->qvel[2]};
            mj_step(model.get(), data.get());

            for (int i = 0; i < 3; ++i)
                acceleration[i] = (data->qvel[i] - velocity[i]) / model->opt.timestep;

            mj_forward(model.get(), data.get());
            const auto truth = qhrr05::ReadContactGroundTruth(model.get(), data.get());
            const auto& m = controller.telemetry();
            double gt = 0;

            for (int i = 0; i < 3; ++i)
                gt -= m.normal[i] * truth[i];

            if (m.phase == MagneticPhase::Free || m.phase == MagneticPhase::Candidate)
                gt = std::sqrt(truth[0] * truth[0] + truth[1] * truth[1] + truth[2] * truth[2]);

            peak = std::max(peak, std::abs(gt));

            if (m.phase == MagneticPhase::Following)
                ++following;

            if (time > 1.5 && time < 2.7 && m.phase == MagneticPhase::Following) {
                sum_gt += gt;
                sum_est += m.estimated_normal_force;
                sum_squared += (gt - 3) * (gt - 3);
                ++count;
            }

            if (step % 10 == 0)
                csv << time << ',' << MagneticPhaseName(m.phase) << ','
                    << MagneticReasonName(m.reason) << ',' << m.estimated_normal_force << ',' << gt
                    << ',' << m.force_command << ',' << m.displacement << ','
                    << data->site_xpos[3 * toe] << ',' << center << ',' << truth[3] << '\n';
        }

        std::cout << "physics following_steps=" << following << " steady_samples=" << count
                  << " mean_gt=" << (count ? sum_gt / count : 0)
                  << " mean_est=" << (count ? sum_est / count : 0)
                  << " RMSE=" << (count ? std::sqrt(sum_squared / count) : 0) << " peak=" << peak
                  << '\n';
        Require(following > 1000 && count > 500, "physics test did not sustain force following");
        Require(controller.telemetry().phase == MagneticPhase::Free,
            "physical contact removal did not release");
        // Functional bounds; force accuracy remains a user-tunable demonstration parameter.
        Require(std::isfinite(sum_gt) && sum_gt / count > .5 && sum_gt / count < 6,
            "invalid measured contact force");

        for (int i = 0; i < mjNWARNING; ++i)
            Require(!data->warning[i].number, "MuJoCo dynamics warning");

        std::cout
            << "PASS: RFOB sign/filter/repeated sample/singularity, FSM, workspace/force/travel bounds; torque motor and condim6 retained\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

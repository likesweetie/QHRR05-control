#include "motion_controllers/magnetic_touch_controller.hpp"
#include "simulation/simulation.hpp"
#include "robot_model/backend.hpp"
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <unistd.h>

void Require(bool v, const char* message) {
    if (!v)
        throw std::runtime_error(message);
}

void CheckGeneralizedMomentum(const char* config_path) {
    auto config = YAML::LoadFile(config_path);
    config["magnetic_touch"]["observer"]["harmonic_compensation"]["enabled"] = false;
    config["magnetic_touch"]["observer"]["friction_velocity_rad_s"] = .01;
    config["magnetic_touch"]["observer"]["maximum_sample_gap_sec"] = .02;
    const auto path = std::filesystem::temp_directory_path() /
        ("qhrr-gm-observer-" + std::to_string(getpid()) + ".yaml");
    std::ofstream(path) << config;
    qhrr05::ReactionForceObserver gm(path.string());
    std::array<double, 3> q{}, v{.2, -.3, .4}, zero{};
    std::array<double, 9> mass{1, 0, 0, 0, 1, 0, 0, 0, 1}, coriolis{};
    const std::array<double, 9> jac{.2, 0, 0, 0, .2, 0, 0, 0, .2};
    gm.Update(1, .001, v, zero, mass, coriolis, jac, zero, zero, zero, q);
    Require(!gm.valid() && gm.warming_up() && qhrr05::numeric::Norm(gm.force()) == 0,
        "GM did not prime a nonzero entry momentum without an impact");
    gm.Update(2, .001, v, zero, mass, coriolis, jac, zero, zero, zero, q);
    Require(gm.valid() && qhrr05::numeric::Norm(gm.force()) < 1e-12,
        "GM reported external force for constant free momentum");

    // M(t) and dq(t) are linear, so p(t) is quadratic and trapezoidal model
    // integration is exact. Nonzero Mdot/C terms are essential for zero bias.
    gm.Reset();
    const std::array<double, 9> mass_rate{.02, .003, 0, .003, .01, -.002, 0, -.002, .03};
    const std::array<double, 3> acceleration{.3, -.2, .1}, initial_velocity=v;
    const std::array<double, 3> gravity{1, 2, 3}, passive{-.2, .1, -.3};
    const std::array<double, 3> friction{.1, .2, .3}, external{2, -1, 3};
    for (int sample = 1; sample <= 300; ++sample) {
        const double t = .002 * sample;
        std::array<double, 3> bias=gravity, motor{};
        for (int i = 0; i < 3; ++i) v[i] = initial_velocity[i] + acceleration[i] * t;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                mass[3*i+j] = (i == j ? 1.0 : 0.0) + mass_rate[3*i+j] * t;
                coriolis[3*i+j] = .5 * mass_rate[3*i+j];
                bias[i] += coriolis[3*i+j] * v[j];
            }
        for (int i = 0; i < 3; ++i) {
            motor[i] = bias[i] - passive[i] + friction[i] * std::tanh(v[i] / .01);
            for (int j = 0; j < 3; ++j)
                motor[i] += mass[3*i+j] * acceleration[j] - jac[3*j+i] * external[j];
        }
        gm.Update(2 * sample, .001, v, motor, mass, coriolis, jac, bias, passive, friction, q);
    }
    Require(gm.valid(), "GM with moving mass did not become valid");
    for (int i = 0; i < 3; ++i)
        Require(std::abs(gm.force()[i] - external[i]) < 1e-8,
            "GM failed Mdot/C compensation, external-force sign, or skipped-sample timing");

    coriolis.fill(0);
    gm.Update(621, .001, v, zero, mass, coriolis, jac, zero, zero, zero, q);
    Require(!gm.valid() && qhrr05::numeric::Norm(gm.force()) == 0,
        "GM kept stale momentum over a long sample gap");
    gm.Update(622, .001, v, zero, mass, coriolis, jac, zero, zero, zero, q);
    Require(!gm.valid() && gm.warming_up(), "GM did not re-prime after a sample gap");
    gm.Update(623, .001, v, zero, mass, coriolis, jac, zero, zero, zero, q);
    Require(gm.valid() && qhrr05::numeric::Norm(gm.force()) < 1e-12,
        "GM reinitialization introduced a false external impulse");
    gm.Update(1, .001, v, zero, mass, coriolis, jac, zero, zero, zero, q);
    Require(!gm.valid() && gm.warming_up(), "GM retained momentum across a sequence rewind");

    const auto motor_path = path.string() + ".motor.yaml";
    std::ofstream(motor_path) << "mit:\n  gear_ratio: 9\n";
    auto h=config["magnetic_touch"]["observer"]["harmonic_compensation"];
    h["enabled"]=true;
    h["motor_config"]=motor_path;
    h["motor_zero_rad"]=YAML::Load("[.4,-.2,.1]");
    h["coefficients_nm"]=YAML::Load("[[.2,-.1,.05,.03],[.03,.06,-.01,.02],[-.02,.04,.03,-.06]]");
    std::ofstream(path) << config;
    qhrr05::ReactionForceObserver compensated(path.string());
    q={.173,.035,-.08};
    v.fill(0);
    mass={1,0,0,0,1,0,0,0,1};
    std::array<double,3> motor{};
    for (int i=0;i<3;++i) {
        const double theta=9*q[i]+h["motor_zero_rad"][i].as<double>();
        const auto coefficients=h["coefficients_nm"][i].as<std::vector<double>>();
        const double ripple=coefficients[0]*std::sin(21*theta)+coefficients[1]*std::cos(21*theta)+
            coefficients[2]*std::sin(36*theta)+coefficients[3]*std::cos(36*theta);
        motor[i]=-(.2*external[i]+ripple);
    }
    for (int i=1;i<=500;++i)
        compensated.Update(i,.001,v,motor,mass,coriolis,jac,zero,zero,zero,q);
    for (int i=0;i<3;++i)
        Require(std::abs(compensated.force()[i]-external[i])<1e-8 &&
            std::abs(compensated.torque()[i]-.2*external[i])<1e-8,
            "GM harmonics used the wrong shaft angle, units or compensation sign");
    std::filesystem::remove(motor_path);
    std::filesystem::remove(path);
    std::cout << "PASS: GM priming, moving M/C, skipped samples, stale reset and 21/36 harmonics\n";
}

void CheckSurfaceFollowing(const char* config_path) {
    auto config = YAML::LoadFile(config_path);
    auto settings = config["magnetic_touch"];
    // Remove observer settling from the direction tests; the normal estimator
    // itself still runs at the configured 2 Hz / 90 degrees per second.
    settings["observer"]["bandwidth_hz"] = 10000;
    settings["maximum_displacement_m"] = .015;
    settings["normal_bandwidth_hz"] = 2;
    settings["normal_max_rate_rad_s"] = std::acos(-1.0) / 2;
    settings["normal_max_deviation_rad"] = std::acos(-1.0) / 6;
    const auto path = std::filesystem::temp_directory_path() /
        ("qhrr-magnetic-surface-" + std::to_string(getpid()) + ".yaml");
    std::ofstream(path) << config;
    qhrr05::MagneticTouchController controller(path.string());
    const std::array<double, 3> q{0, .9, -1.8}, zero{}, qmin{-1, -1, -3}, qmax{1, 3, -.5};
    const std::array<double, 9> mass{.1, 0, 0, 0, .1, 0, 0, 0, .1};
    const std::array<double, 9> jac{.2, 0, 0, 0, .2, 0, 0, 0, .2};
    std::array<double, 3> p{.2, .14, .18}, force{-3, 0, 0}, velocity{0, .5, 0}, reference{}, torque{};
    double time = 0;
    std::uint64_t sequence = 0;
    const auto tick = [&]() {
        time += .001;
        std::array<double, 3> motor{};
        for (int i = 0; i < 3; ++i) motor[i] = -.2 * force[i];
        controller.Compute(time, q, velocity, motor, zero, zero, zero, zero,
            mass, {}, jac, p, qmin, qmax, ++sequence, .001, 3, reference, torque);
    };
    controller.Start(time, p);
    for (int i = 0; i < 500; ++i) tick();
    Require(controller.telemetry().phase == MagneticPhase::Following, "surface trial did not contact");

    // Forty millimetres of tangential travel exceeds the normal stroke limit,
    // but must neither release the contact nor create tangential spring/damping.
    velocity[1] = .5;
    for (int i = 0; i < 400; ++i) {
        p[1] += .0001;
        tick();
        Require(controller.telemetry().phase == MagneticPhase::Following,
            "tangential motion incorrectly consumed the normal stroke");
        Require(std::abs(controller.telemetry().toe_reference[1] - p[1]) < 1e-12 &&
            std::abs(torque[1]) < 1e-12 && std::abs(torque[2]) < 1e-12,
            "contact command resisted tangential motion");
    }

    // Slowly turn the surface by 30 degrees while sliding along its tangent.
    auto old_normal = controller.telemetry().normal;
    for (int i = 1; i <= 500; ++i) {
        const double angle = i / 500.0 * std::acos(-1.0) / 6;
        force = {-3 * std::cos(angle), -3 * std::sin(angle), 0};
        p[0] -= .00001 * std::sin(angle);
        p[1] += .00001 * std::cos(angle);
        tick();
        const auto& normal = controller.telemetry().normal;
        const double turn = std::acos(std::clamp(qhrr05::numeric::Dot(old_normal, normal), -1.0, 1.0));
        Require(std::abs(qhrr05::numeric::Norm(normal) - 1) < 1e-12 &&
            turn <= std::acos(-1.0) / 2 * .001 + 1e-9,
            "normal is not unit length or exceeds its angular-rate limit");
        Require(controller.telemetry().phase == MagneticPhase::Following &&
            std::abs(torque[0] * normal[1] - torque[1] * normal[0]) < 1e-12,
            "curved contact was lost or produced tangential contact torque");
        old_normal = normal;
    }
    Require(controller.telemetry().normal[1] > .35,
        "normal stayed fixed instead of following the curved surface");

    const auto before_step = controller.telemetry().normal;
    const double small_jump = std::atan2(before_step[1], before_step[0]) + std::acos(-1.0) / 9;
    force = {-3 * std::cos(small_jump), -3 * std::sin(small_jump), 0};
    tick();
    const double limited_angle = std::acos(std::clamp(
        qhrr05::numeric::Dot(before_step, controller.telemetry().normal), -1.0, 1.0));
    Require(std::abs(limited_angle - std::acos(-1.0) / 2 * .001) < 1e-9,
        "normal rotation did not honor the angular rate cap on a valid direction step");

    // A single 60-degree direction jump is rejected, even with strong force.
    const auto held_normal = controller.telemetry().normal;
    const double angle = std::atan2(held_normal[1], held_normal[0]) + std::acos(-1.0) / 3;
    force = {-3 * std::cos(angle), -3 * std::sin(angle), 0};
    tick();
    Require(controller.telemetry().normal == held_normal, "large normal outlier was accepted");

    // Below the recovery threshold the estimator freezes, despite valid contact.
    const double low_angle = std::atan2(held_normal[1], held_normal[0]) + .1;
    force = {-.7 * std::cos(low_angle), -.7 * std::sin(low_angle), 0};
    for (int i = 0; i < 10; ++i) tick();
    Require(controller.telemetry().normal == held_normal &&
        controller.telemetry().phase == MagneticPhase::Following,
        "weak contact changed the estimated normal");

    // Pure tangential force must not rotate into an apparent normal contact.
    force = {3 * held_normal[1], -3 * held_normal[0], 0};
    tick();
    Require(controller.telemetry().phase == MagneticPhase::Releasing,
        "tangential force hid contact loss");
    const double frozen_displacement = controller.telemetry().displacement;
    for (int i = 0; i < 50; ++i) tick();
    Require(controller.telemetry().normal == held_normal &&
        controller.telemetry().displacement == frozen_displacement,
        "uncertain contact updated normal or admittance");
    for (int i = 0; i < 100; ++i) tick();
    Require(controller.telemetry().phase == MagneticPhase::Free &&
        controller.telemetry().reason == MagneticReason::ContactLost,
        "tangential force prevented release");

    for (int direction : {-1, 1}) {
        p = {.2, .14, .18}; force = {-3, 0, 0};
        controller.Start(time, p);
        for (int i = 0; i < 500; ++i) tick();
        p[0] += direction * .016;
        tick();
        Require(controller.telemetry().phase == MagneticPhase::Free &&
            controller.telemetry().reason == MagneticReason::StrokeLimit,
            "signed normal stroke limit was not enforced");
    }

    for (const auto& [key, value] : std::array<std::pair<const char*, double>, 4>{{
        {"normal_bandwidth_hz", 0}, {"normal_max_rate_rad_s", -1},
        {"normal_max_deviation_rad", 2}, {"normal_bandwidth_hz", std::numeric_limits<double>::quiet_NaN()}}}) {
        auto invalid = YAML::Clone(config);
        invalid["magnetic_touch"][key] = value;
        std::ofstream(path) << invalid;
        bool rejected = false;
        try { qhrr05::MagneticTouchController bad(path.string()); }
        catch (const std::exception&) { rejected = true; }
        Require(rejected, "invalid normal tracking configuration was accepted");
    }
    std::filesystem::remove(path);
    std::cout << "PASS: adaptive normal, tangent anchor, normal-only torque, force gating and normal stroke\n";
}

int main(int argc, char** argv) {
    try {
        Require(argc == 3, "expected config and scene");
        CheckGeneralizedMomentum(argv[1]);
        qhrr05::ReactionForceObserver observer(argv[1]);
        std::array<double, 3> joint_position{};
        std::array<double, 3> joint_velocity{};
        std::array<double, 3> joint_acceleration{};
        std::array<double, 3> motor_torque{};
        std::array<double, 3> gravity_torque{};
        std::array<double, 3> bias_torque{};
        std::array<double, 3> passive_torque{};
        std::array<double, 3> friction_limit{};
        std::array<double, 9> mass_matrix{};
        std::array<double, 9> coriolis_matrix{};
        std::array<double, 9> jacobian{};
        std::array<double, 3> toe_position{};
        std::array<double, 3> joint_min{};
        std::array<double, 3> joint_max{};
        std::uint64_t measurement_sequence = 0;
        double measurement_dt = .001;
        jacobian = {.18, .02, -.01, .04, .25, .03, .01, -.02, .22};
        mass_matrix = {.1, 0, 0, 0, .2, 0, 0, 0, .3};
        bias_torque = {1, 2, 3};
        joint_acceleration = {.3, -.2, .1};
        passive_torque = {-.2, .1, -.3};
        const double expected[] = {2, -1, 3};

        for (int i = 0; i < 3; ++i) {
            motor_torque[i] =
                mass_matrix[3 * i + i] * joint_acceleration[i] + bias_torque[i] - passive_torque[i];

            for (int j = 0; j < 3; ++j)
                motor_torque[i] -= jacobian[3 * j + i] * expected[j];
        }

        for (int i = 1; i <= 300; ++i) {
            measurement_sequence = i;
            for (int axis = 0; axis < 3; ++axis)
                joint_velocity[axis] += joint_acceleration[axis] * measurement_dt;
            observer.Update(measurement_sequence,
                measurement_dt,
                joint_velocity,
                motor_torque,
                mass_matrix,
                coriolis_matrix,
                jacobian,
                bias_torque,
                passive_torque,
                friction_limit, joint_position);
        }

        Require(observer.valid(), "RFOB rejected a full-rank Jacobian");

        for (int i = 0; i < 3; ++i)
            Require(std::abs(observer.force()[i] - expected[i]) < .001,
                "RFOB residual force/sign is wrong");

        const auto previous = observer.force();
        motor_torque.fill(100);
        observer.Update(measurement_sequence,
            measurement_dt,
            joint_velocity,
            motor_torque,
            mass_matrix,
            coriolis_matrix,
            jacobian,
            bias_torque,
            passive_torque,
            friction_limit, joint_position);
        Require(observer.force() == previous, "RFOB processed a repeated measurement");
        jacobian.fill(0);
        observer.Update(measurement_sequence,
            measurement_dt,
            joint_velocity,
            motor_torque,
            mass_matrix,
            coriolis_matrix,
            jacobian,
            bias_torque,
            passive_torque,
            friction_limit, joint_position);
        Require(!observer.valid(), "RFOB accepted a singular pose");

        qhrr05::MagneticTouchController magnetic(argv[1]);
        joint_position = joint_velocity = joint_acceleration = motor_torque = {};
        gravity_torque = bias_torque = passive_torque = friction_limit = {};
        mass_matrix = jacobian = {};
        toe_position = joint_min = joint_max = {};
        measurement_sequence = 0;
        measurement_dt = .001;
        jacobian = {.2, 0, 0, 0, .2, 0, 0, 0, .2};
        mass_matrix = {.1, 0, 0, 0, .1, 0, 0, 0, .1};
        joint_position = {0, .9, -1.8};
        gravity_torque = bias_torque = {1, 2, -.5};
        toe_position = {.2, .14, .18};
        joint_min = {-1, -1, -3};
        joint_max = {1, 3, -.5};
        std::array<double, 3> reference{}, torque{};


        magnetic.Start(0, toe_position);
        double release_position = 0, release_start = -1;

        for (int i = 1; i <= 700; ++i) {
            measurement_sequence = i;
            const double external = i > 200 && i < 550 ? -2.0 : 0.0;
            motor_torque = gravity_torque;
            motor_torque[0] -= .2 * external;
            magnetic.Compute(i * .001,
                joint_position,
                joint_velocity,
                motor_torque,
                gravity_torque,
                bias_torque,
                passive_torque,
                friction_limit,
                mass_matrix,
                coriolis_matrix,
                jacobian,
                toe_position,
                joint_min,
                joint_max,
                measurement_sequence,
                measurement_dt,
                3, reference, torque);
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
                Require(
                    torque[0] > gravity_torque[0], "contact force did not affect torque target");
            }

            if (i == 690)
                Require(telemetry.phase == MagneticPhase::Free &&
                            telemetry.reason == MagneticReason::ContactLost,
                    "contact release failed");
        }

        toe_position[0] = 1;
        magnetic.Compute(.701,
            joint_position,
            joint_velocity,
            motor_torque,
            gravity_torque,
            bias_torque,
            passive_torque,
            friction_limit,
            mass_matrix,
            coriolis_matrix,
            jacobian,
            toe_position,
            joint_min,
            joint_max,
            measurement_sequence,
            measurement_dt,
            3, reference, torque);
        Require(magnetic.telemetry().reason == MagneticReason::Workspace,
            "workspace exit was not reported");

        toe_position = {.2, .14, .18};
        qhrr05::MagneticTouchController bounded(argv[1]);
        bounded.Start(1, toe_position);
        motor_torque = gravity_torque;
        motor_torque[0] += .4;

        for (int i = 1; i <= 300; ++i) {
            measurement_sequence = 800 + i;
            bounded.Compute(1 + i * .001,
                joint_position,
                joint_velocity,
                motor_torque,
                gravity_torque,
                bias_torque,
                passive_torque,
                friction_limit,
                mass_matrix,
                coriolis_matrix,
                jacobian,
                toe_position,
                joint_min,
                joint_max,
                measurement_sequence,
                measurement_dt,
                3, reference, torque);
        }

        Require(bounded.telemetry().phase == MagneticPhase::Following,
            "bounded trial did not enter following");
        toe_position[0] += .02;
        bounded.Compute(1.301,
            joint_position,
            joint_velocity,
            motor_torque,
            gravity_torque,
            bias_torque,
            passive_torque,
            friction_limit,
            mass_matrix,
            coriolis_matrix,
            jacobian,
            toe_position,
            joint_min,
            joint_max,
            measurement_sequence,
            measurement_dt,
            3, reference, torque);
        Require(bounded.telemetry().phase == MagneticPhase::Free &&
                    bounded.telemetry().reason == MagneticReason::StrokeLimit,
            "actual following distance limit did not release");
        toe_position = {.2, .14, .18};
        joint_position[0] = .99;
        bounded.Compute(1.302,
            joint_position,
            joint_velocity,
            motor_torque,
            gravity_torque,
            bias_torque,
            passive_torque,
            friction_limit,
            mass_matrix,
            coriolis_matrix,
            jacobian,
            toe_position,
            joint_min,
            joint_max,
            measurement_sequence,
            measurement_dt,
            3, reference, torque);
        Require(bounded.telemetry().reason == MagneticReason::JointLimit,
            "joint limit was not reported");
        joint_position[0] = 0;
        jacobian.fill(0);
        bounded.Compute(1.303,
            joint_position,
            joint_velocity,
            motor_torque,
            gravity_torque,
            bias_torque,
            passive_torque,
            friction_limit,
            mass_matrix,
            coriolis_matrix,
            jacobian,
            toe_position,
            joint_min,
            joint_max,
            measurement_sequence,
            measurement_dt,
            3, reference, torque);
        Require(bounded.telemetry().reason == MagneticReason::ObserverInvalid,
            "invalid observer did not disable following");
        gravity_torque[0] = 1000;
        bounded.Compute(1.304,
            joint_position,
            joint_velocity,
            motor_torque,
            gravity_torque,
            bias_torque,
            passive_torque,
            friction_limit,
            mass_matrix,
            coriolis_matrix,
            jacobian,
            toe_position,
            joint_min,
            joint_max,
            measurement_sequence,
            measurement_dt,
            3, reference, torque);
        Require(bounded.telemetry().torque_saturated && torque[0] == 23.7,
            "magnetic torque saturation failed");
        bool target_rejected = false;

        try {
            bounded.Compute(1.305,
                joint_position,
                joint_velocity,
                motor_torque,
                gravity_torque,
                bias_torque,
                passive_torque,
                friction_limit,
                mass_matrix,
                coriolis_matrix,
                jacobian,
                toe_position,
                joint_min,
                joint_max,
                measurement_sequence,
                measurement_dt,
                6, reference, torque);
        } catch (const std::exception&) {
            target_rejected = true;
        }

        Require(target_rejected, "out-of-range target force was accepted");
        CheckSurfaceFollowing(argv[1]);

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
        auto backend = qhrr05::MakeMujocoBackend(argv[1]);
        controller.Start(0, toe_position);
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
            qhrr05::UpdateControlIO(
                shm, model.get(), data.get(), gravity.get(), reference, step, nullptr, true);
            const auto model_state = shm.model[shm.model_sequence % kShmBufferCount];
            const auto motors = shm.motors[shm.motor_sequence % kShmBufferCount];
            std::array<double, 3> physical_position, physical_velocity;

            for (int i = 0; i < 3; ++i) {
                physical_position[i] = motors.feedback[i].position;
                physical_velocity[i] = motors.feedback[i].velocity;
            }

            controller.Compute(time,
                physical_position,
                physical_velocity,
                motors.motor_torque,
                model_state.gravity_torque,
                model_state.bias_torque,
                model_state.passive_torque,
                model_state.friction_limit,
                model_state.mass,
                backend->CoriolisMatrix(physical_position, physical_velocity),
                model_state.jacobian,
                model_state.toe_position,
                model_state.joint_min,
                model_state.joint_max,
                step,
                model->opt.timestep,
                3, reference, torque);
            // Model the enabled MIT motor at v_des=0. The motion controller's
            // torque target excludes joint damping, which the motor supplies.
            for (int i = 0; i < 3; ++i)
                data->ctrl[i] = std::clamp(
                    torque[i] - controller.kd_motor()[i] * physical_velocity[i],
                    model->actuator_ctrlrange[2 * i],
                    model->actuator_ctrlrange[2 * i + 1]);

            mj_step(model.get(), data.get());

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

#include "simulation/interaction.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
}

int main(int argc, char** argv) {
    try {
        Require(argc == 2, "expected physics model");
        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        mj_forward(model.get(), data.get());
        const std::string session = "/qhrr-interaction-" + std::to_string(getpid());
        qhrr05::SimulationInteraction interaction(session, model.get(), data.get());
        qhrr05::simulation_ipc::Client controller(session, true);
        controller.Poll();
        interaction.Poll(false);
        SimulationData snapshot;
        snapshot.published_ns = shm_utils::MonotonicNs();
        interaction.PublishState(snapshot);
        controller.Poll();

        SimulationCommand command;
        command.action = SimulationAction::Perturb;
        auto& target = command.perturb;
        target.owner = 7;
        target.active = mjPERT_TRANSLATE;
        const char calf[] = "FL_calf";
        std::copy(std::begin(calf), std::end(calf), target.body.begin());
        const int body = mj_name2id(model.get(), mjOBJ_BODY, calf);
        Require(body > 0, "calf body missing");
        std::copy_n(data->xipos + body * 3, 3, target.reference_position.begin());
        target.reference_position[0] += .03;
        std::copy_n(data->xquat + body * 4, 4, target.reference_quaternion.begin());
        controller.SendApproved(command);
        interaction.Poll(false);
        interaction.Apply(false);
        interaction.Publish(snapshot, false);
        Require(snapshot.perturb_active == mjPERT_TRANSLATE && snapshot.perturb_owner == 7,
            "approved perturbation was not applied");
        Require(std::abs(snapshot.perturb_wrench[0]) > .01, "physical perturbation force is zero");
        std::this_thread::sleep_for(std::chrono::milliseconds(550));
        interaction.Apply(false);
        interaction.Publish(snapshot, false);
        Require(snapshot.perturb_active == 0 && snapshot.perturb_owner == 0,
            "stalled viewer retained its perturbation lease");
        for (double force : snapshot.perturb_wrench)
            Require(force == 0, "expired perturbation left a force behind");

        target.body.fill(0);
        const char object[] = "touch_object";
        std::copy(std::begin(object), std::end(object), target.body.begin());
        const int mocap = model->body_mocapid[mj_name2id(model.get(), mjOBJ_BODY, object)];
        Require(mocap >= 0, "object is not mocap");
        target.reference_position = {.1, .2, .3};
        target.reference_quaternion = {std::sqrt(.5), 0, 0, std::sqrt(.5)};
        target.active = mjPERT_TRANSLATE;
        controller.SendApproved(command);
        interaction.Poll(true);
        interaction.Apply(true);
        for (int axis = 0; axis < 3; ++axis)
            Require(std::abs(data->mocap_pos[mocap * 3 + axis] - target.reference_position[axis]) <
                        1e-9,
                "mocap position did not follow approved target");
        target.active = mjPERT_ROTATE;
        controller.SendApproved(command);
        interaction.Poll(true);
        interaction.Apply(true);
        for (int axis = 0; axis < 4; ++axis)
            Require(std::abs(data->mocap_quat[mocap * 4 + axis] -
                             target.reference_quaternion[axis]) < 1e-9,
                "mocap rotation did not follow approved target");
        target.active = 0;
        controller.SendApproved(command);
        interaction.Poll(true);
        interaction.Apply(true);
        interaction.Publish(snapshot, true);
        Require(snapshot.perturb_active == 0, "explicit perturbation release failed");
        std::cout
            << "PASS: physical force, expired lease clears force, mocap pose and explicit release\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

#pragma once

#include "motion_controllers/ball_following_controller.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <string>
#include <vector>

namespace qhrr05::viewer {
class WorkspaceOverlay {
public:
    ~WorkspaceOverlay() {
        cancelled_.store(true, std::memory_order_relaxed);

        if (pending_.valid())
            pending_.wait();
    }

    void Toggle(const std::string& config_path) {
        visible_ = !visible_;
        std::cout << "[workspace] " << (visible_ ? "ON" : "OFF") << std::endl;

        if (!visible_)
            return;

        error_.clear();

        if (geometry_.empty() && !pending_.valid()) {
            pending_ = std::async(std::launch::async, [config_path, cancel = &cancelled_] {
                BallFollowingController workspace(config_path, cancel);
                const auto surface = workspace.WorkspaceSurface();
                std::vector<mjvGeom> result;
                result.reserve(surface.size() * 2);

                for (const auto& triangle : surface)
                    AddTriangle(result, triangle);

                return result;
            });
        }
    }

    std::string Status() const {
        if (!visible_)
            return "OFF";

        if (!error_.empty())
            return "ERROR: " + error_;

        return geometry_.empty() ? "BUILDING..." : "ON - condition-filtered FK workspace";
    }

    void Draw(mjvScene& scene) {
        if (pending_.valid() && pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                geometry_ = pending_.get();
                std::cout << "[workspace] ready: " << geometry_.size()
                          << " transparent triangles" << std::endl;
            } catch (const std::exception& error) {
                error_ = error.what();
                std::cerr << "[workspace] ERROR: " << error_ << std::endl;
            }
        }

        if (!visible_ || !error_.empty())
            return;

        if (geometry_.size() > static_cast<std::size_t>(scene.maxgeom - scene.ngeom)) {
            error_ = "workspace mesh exceeds scene capacity; reduce workspace_grid_points";
            std::cerr << "[workspace] ERROR: " << error_ << std::endl;
            return;
        }

        std::copy(geometry_.begin(), geometry_.end(), scene.geoms + scene.ngeom);
        scene.ngeom += static_cast<int>(geometry_.size());
    }

private:
    static void AddTriangle(std::vector<mjvGeom>& result, const std::array<double, 9>& triangle) {
        const double* vertices[] = {triangle.data(), triangle.data() + 3, triangle.data() + 6};
        int longest = 0;

        for (int edge = 1; edge < 3; ++edge)
            if (mju_dist3(vertices[edge], vertices[(edge + 1) % 3]) >
                mju_dist3(vertices[longest], vertices[(longest + 1) % 3]))
                longest = edge;

        const auto* a = vertices[longest];
        const auto* b = vertices[(longest + 1) % 3];
        const auto* c = vertices[(longest + 2) % 3];
        double direction[3], delta[3], foot[3];
        mju_sub3(direction, b, a);
        mju_normalize3(direction);
        mju_sub3(delta, c, a);
        const double projection = mju_dot3(delta, direction);
        mju_copy3(foot, a);
        mju_addToScl3(foot, direction, projection);

        // MuJoCo's rendering triangle is right-angled. Split at the altitude of
        // the longest edge, retaining an orthonormal frame and outward winding.
        const double* arms[2][2] = {{c, a}, {b, c}};

        for (const auto& arm : arms) {
            double u[3], v[3], normal[3];
            mju_sub3(u, arm[0], foot);
            mju_sub3(v, arm[1], foot);
            const double width = mju_normalize3(u), height = mju_normalize3(v);

            if (width < 1e-10 || height < 1e-10)
                continue;  // Zero-area part of a triangle whose altitude meets a vertex.

            mju_cross(normal, u, v);
            mju_normalize3(normal);
            const double mat[] = {u[0], v[0], normal[0], u[1], v[1], normal[1], u[2], v[2], normal[2]};
            const double size[] = {width, height, 0};
            const float rgba[] = {.25f, .85f, .5f, .10f};
            mjvGeom geom;
            mjv_initGeom(&geom, mjGEOM_TRIANGLE, size, foot, mat, rgba);
            geom.category = mjCAT_DECOR;
            geom.objtype = mjOBJ_UNKNOWN;
            geom.objid = geom.segid = -1;
            geom.specular = geom.reflectance = 0;
            geom.emission = .25f;
            result.push_back(geom);
        }
    }

    bool visible_ = false;
    std::atomic<bool> cancelled_{false};
    std::future<std::vector<mjvGeom>> pending_;
    std::vector<mjvGeom> geometry_;
    std::string error_;
};
}  // namespace qhrr05::viewer

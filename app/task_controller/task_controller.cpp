#include "task_controller.hpp"
#include <fcntl.h>
#include <cerrno>
#include <sys/socket.h>
#include <unistd.h>

namespace qhrr05 {
TaskController::TaskController(const YAML::Node& settings, int fd)
    : fd_(fd), homing_(std::make_unique<HomingController>(settings)) {
    if (fd < 0 || ::fcntl(fd, F_GETFD) < 0) throw std::runtime_error("invalid control descriptor");
}
TaskController::~TaskController() { ::close(fd_); }

bool TaskController::ReadInput() {
    ssize_t size;
    do { size = ::recv(fd_, input_.data(), sizeof(input_), MSG_TRUNC); } while (size < 0 && errno == EINTR);
    if (size == 0) return false;
    if (size != sizeof(input_)) throw std::runtime_error("joint input read failed or has the wrong size");
    return true;
}
void TaskController::Control() {
    homing_->Compute(input_[0], input_.data()+1, input_.data()+4, input_.data()+7,
                     output_.data(), output_.data()+3);
}
void TaskController::WriteOutput() {
    ssize_t size;
    do { size = ::send(fd_, output_.data(), sizeof(output_), MSG_NOSIGNAL); } while (size < 0 && errno == EINTR);
    if (size != sizeof(output_)) throw std::runtime_error("joint output write failed");
}
}  // namespace qhrr05

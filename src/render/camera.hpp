#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>

namespace gs::render {

struct Camera {
    glm::vec3 pos{0.0f, 1000.0f, 5000.0f};   // world-units
    float yaw = -90.0f;
    float pitch = 0.0f;
    float fov = 45.0f;
    float nearPlane = 0.1f;
    float farPlane = 500000.0f;

    glm::vec3 front() const {
        const float yr = glm::radians(yaw), pr = glm::radians(pitch);
        return glm::normalize(glm::vec3(
            std::cos(yr) * std::cos(pr), std::sin(pr), std::sin(yr) * std::cos(pr)));
    }
    glm::vec3 right() const { return glm::normalize(glm::cross(front(), glm::vec3(0, 1, 0))); }
    glm::vec3 up()    const { return glm::normalize(glm::cross(right(), front())); }

    void rotate(float dx, float dy) {
        yaw += dx;
        pitch = std::clamp(pitch + dy, -89.0f, 89.0f);
    }
    glm::mat4 view() const { return glm::lookAt(pos, pos + front(), glm::vec3(0, 1, 0)); }
    glm::mat4 projection(float aspect) const {
        return glm::perspective(glm::radians(fov), aspect, nearPlane, farPlane);
    }
};

} // namespace gs::render

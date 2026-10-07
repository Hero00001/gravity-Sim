#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
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
    glm::mat4 viewProj(float aspect) const { return projection(aspect) * view(); }

    // Project a world-unit point to normalized device coords (ndc.xy in [-1,1], ndc.z in [-1,1] if visible).
    glm::vec3 projectToNDC(const glm::vec3& worldUnits, float aspect) const {
        glm::vec4 clip = viewProj(aspect) * glm::vec4(worldUnits, 1.0f);
        if (clip.w <= 1e-6f) return glm::vec3(0.0f, 0.0f, 2.0f);
        return glm::vec3(clip) / clip.w;
    }
    // Unproject a cursor NDC coordinate into a world-space ray direction (unit length).
    glm::vec3 unprojectDir(float ndcX, float ndcY, float aspect) const {
        glm::mat4 inv = glm::inverse(viewProj(aspect));
        glm::vec4 nearP = inv * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
        glm::vec4 farP  = inv * glm::vec4(ndcX, ndcY,  1.0f, 1.0f);
        nearP /= nearP.w; farP /= farP.w;
        return glm::normalize(glm::vec3(farP - nearP));
    }
    // Intersect ray (origin, dir) with plane (planePoint, planeNormal); returns origin if parallel.
    static glm::vec3 rayPlaneIntersect(const glm::vec3& origin, const glm::vec3& dir,
                                      const glm::vec3& planePoint, const glm::vec3& planeNormal) {
        const float denom = glm::dot(dir, planeNormal);
        if (std::fabs(denom) < 1e-6f) return origin;
        const float t = glm::dot(planePoint - origin, planeNormal) / denom;
        return origin + dir * t;
    }

    // Follow mode: lock camera to a body (keeps the offset captured at startFollow).
    void startFollow(const glm::vec3& targetUnits) { following_ = true; followOffset_ = pos - targetUnits; }
    void stopFollow() { following_ = false; }
    bool isFollowing() const { return following_; }
    void updateFollow(const glm::vec3& targetUnits) { if (following_) pos = targetUnits + followOffset_; }

    // Auto-frame a scene: place the camera to look at the origin from a diagonal, with
    // near/far planes scaled to the scene's reference radius (spec #3 camera framing).
    void frameScene(double radiusUnits) {
        const float r = static_cast<float>(radiusUnits);
        pos = glm::vec3(0.0f, r * 0.45f, r * 2.2f);
        yaw = -90.0f;
        pitch = -glm::degrees(std::atan2(r * 0.45f, r * 2.2f));
        fov = 45.0f;
        nearPlane = std::max(0.1f, r * 0.0008f);
        farPlane  = std::max(500000.0f, r * 6.0f);
        following_ = false;
    }

private:
    bool following_ = false;
    glm::vec3 followOffset_{0.0f, 0.0f, 0.0f};
};

} // namespace gs::render

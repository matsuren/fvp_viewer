#include "gl_camera.hpp"

#include <spdlog/spdlog.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

namespace fvp {
GLCamera::GLCamera()
    : theta(glm::radians(-95.0f)),
      phi(glm::radians(30.0f)),
      zoom(290),
      lookat_position(glm::vec3(0.0f, 0.0f, 1.0f)),
      DRAG_FACTOR(0.004f),
      ZOOM_FACTOR(5),
      isAnimation(false),
      ANIMATION_SPEED(glm::pi<float>() / 20.0f),
      world_light(glm::vec4(5.0f, 5.0f, -4.0f, 1.0f)) {
  spdlog::info("initialize GLCamera : ");
};

// update
void GLCamera::update(float t) {
  static float t_prev = 0.0f;
  static int direction = 1;
  if (!isAnimation) return;

  float t_delta = t - t_prev;
  if (t_prev == 0.0f) t_delta = 0.0f;
  t_prev = t;

  theta += direction * ANIMATION_SPEED * t_delta;
  if (abs(theta) > 2 * glm::two_pi<float>() / 3.0 ||
      abs(theta) < glm::radians(120.0f))
    direction *= -1;
}
// Get GL camera view matrix
glm::mat4 GLCamera::getViewMat() {
  // Range for phi angle
  constexpr float PHI_MIN = 0.0f;
  constexpr float PHI_MAX = float(M_PI / 2.0f - 0.00001f);
  if (phi < PHI_MIN) {
    phi = PHI_MIN;
  }
  if (phi > PHI_MAX) {
    phi = PHI_MAX;
  }

  const float distance = zoom / 100.0f;
  // vec3 cameraPos = vec3(2.0f, 1.0f, 0.0f);
  glm::vec3 cameraPos =
      glm::vec3(distance * cos(theta) * cos(phi) + lookat_position.x,
                distance * sin(theta) * cos(phi) + lookat_position.y,
                (distance * sin(phi) + lookat_position.z));
  glm::mat4 view =
      glm::lookAt(cameraPos, lookat_position, glm::vec3(0.0f, 0.0f, 1.0f));
  return view;
}

// scroll
void GLCamera::scrollCallback(double x, double y) {
  zoom += ZOOM_FACTOR * static_cast<int>(y);
  if (zoom < 0) zoom = 0;
}
// mouse button
void GLCamera::mouseCursorCallback(const double x, const double y,
                                   const std::string action) {
  // save state
  static bool is_dragged = false;
  static glm::vec2 prev_mouse_pos;
  static float prev_angle, prev_phi;

  if (action == "PRESS") {
    is_dragged = true;
    prev_mouse_pos.x = float(x);
    prev_mouse_pos.y = float(y);
    prev_angle = theta;
    prev_phi = phi;
  } else if (action == "RELEASE") {
    is_dragged = false;
    spdlog::debug("angle: {} [rad] ({} deg)", theta, glm::degrees(theta));
    spdlog::debug("phi: {} [rad] ({} deg)", phi, glm::degrees(phi));
    spdlog::debug("zoom: {}", zoom);
  } else if (action == "CURSOR_MOVE") {
    if (!is_dragged) return;
    theta = prev_angle - (float(x) - prev_mouse_pos.x) * DRAG_FACTOR;
    phi = prev_phi + (float(y) - prev_mouse_pos.y) * DRAG_FACTOR;
  } else {
    spdlog::warn("Unknow action: {}", action);
  }
}

void GLCamera::diffAngle(double diff_angle) { theta += diff_angle; };
void GLCamera::diffPhi(double diff_phi) { theta += diff_phi; };
// light position
void GLCamera::setWorldLightPosition(const glm::vec4 &light_position) {
  world_light = light_position;
}
glm::vec4 GLCamera::getWorldLightPosition() const { return world_light; }

}  // namespace fvp

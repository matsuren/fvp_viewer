#pragma once
#include <glm/glm.hpp>
#include <string>
namespace fvp {
class GLCamera {
 public:
  GLCamera();

  // update
  void update(float t);
  // Get GL camera view matrix
  glm::mat4 getViewMat();

  // Callback function to move the camera position
  // scroll
  void scrollCallback(double x, double y);
  // mouse button. Action:"PRESS", "RELEASE", "CURSOR_MOVE"
  void mouseCursorCallback(const double x, const double y,
                           const std::string action);

  // light position
  void setWorldLightPosition(const glm::vec4 &light_position);
  glm::vec4 getWorldLightPosition() const;

  void diffAngle(double diff_angle);
  void diffPhi(double diff_phi);

 private:
  // Current OpenGL camera position
  float theta, phi;
  int zoom;
  // Where to look
  const glm::vec3 lookat_position;

  // How fast does the camera move
  const float DRAG_FACTOR;
  const int ZOOM_FACTOR;

  // Animation
  bool isAnimation;
  const float ANIMATION_SPEED;

  // World light
  glm::vec4 world_light;
};
}  // namespace fvp

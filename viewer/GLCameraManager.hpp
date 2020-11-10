#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
using glm::mat4;
using glm::vec2;
using glm::vec3;
using glm::vec4;

// singleton GLCameraManager CLASS
class GLCameraManager {
 private:
  GLCameraManager()
      : angle(glm::radians(-95.0f)),
        phi(glm::radians(30.0f)),
        tPrev(0.0f),
        rotSpeed(glm::pi<float>() / 20.0f),
        isAnimation(false),
        phi_min(0.0f),
        phi_max(M_PI / 2.0f - 0.00001f) {
    std::cout << "initialize GLCameraManager : " << std::endl;
  };
  ~GLCameraManager() = default;

 public:
  GLCameraManager(const GLCameraManager &) = delete;
  GLCameraManager &operator=(const GLCameraManager &) = delete;
  GLCameraManager(GLCameraManager &&) = delete;
  GLCameraManager &operator=(GLCameraManager &&) = delete;

  static GLCameraManager &getInstance() {
    static GLCameraManager inst;
    return inst;
  }

  // update
  void update(float t) {
    if (!isAnimation) return;

    float deltaT = t - tPrev;
    if (tPrev == 0.0f) deltaT = 0.0f;
    tPrev = t;

    angle += rotSpeed * deltaT;
    if (abs(angle) > 2 * glm::two_pi<float>() / 3.0 ||
        abs(angle) < glm::radians(120.0f))
      rotSpeed *= -1;
    // if (abs(angle) > 2*glm::two_pi<float>())
    //	rotSpeed *= -1;
  }
  // update
  mat4 getViewMat() {
    if (phi < phi_min) {
      phi = phi_min;
      prev_phi = phi;
    }
    if (phi > phi_max) {
      phi = phi_max;
      prev_phi = phi;
    }

    static int plus_minus = 1;
    float distance = zoom / 100.0f;
    // vec3 cameraPos = vec3(2.0f, 1.0f, 0.0f);
    vec3 cameraPos =
        vec3(distance * cos(angle) * cos(phi) + lookat_position.x,
             distance * sin(angle) * cos(phi) + lookat_position.y,
             plus_minus * (distance * sin(phi) + lookat_position.z));
    view = glm::lookAt(cameraPos, lookat_position,
                       vec3(0.0f, 0.0f, plus_minus * 1.0f));
    return view;
  }

  // scroll
  void scrollCallback(double x, double y) {
    zoom += zoom_sensitive * static_cast<int>(y);
    if (zoom < 0) zoom = 0;
  }
  // mouse button
  void pressedLeftButton(double x, double y) {
    isDragged = true;
    prev_mouse_pos.x = float(x);
    prev_mouse_pos.y = float(y);
    prev_angle = angle;
    prev_phi = phi;
  }
  // mouse button
  void releasedLeftButton(double x, double y) {
    isDragged = false;
    float degree = glm::degrees(angle);
    std::cout << "angle : " << angle << ",  " << degree << " degree"
              << std::endl;
    degree = glm::degrees(phi);
    std::cout << "phi : " << phi << ",  " << degree << " degree" << std::endl;
    std::cout << "zoom : " << zoom << std::endl;
  }
  // cursor position
  void cursorPositionCallback(double x, double y) {
    if (!isDragged) return;
    angle = prev_angle - (float(x) - prev_mouse_pos.x) * drag_sensitive;
    phi = prev_phi + (float(y) - prev_mouse_pos.y) * drag_sensitive;
  }

  // light position
  void setWorldLightPosition(vec4 &light_position) {
    world_light = light_position;
  }
  vec4 getWorldLightPosition() const { return world_light; }

 public:
  float angle, phi;
  float phi_min, phi_max;

 private:
  float tPrev, rotSpeed;
  mat4 view;
  bool isAnimation;
  int zoom = 290;
  vec3 lookat_position = vec3(0.0f, 0.0f, 1.0f);

  bool isDragged = false;
  vec2 prev_mouse_pos;
  float prev_angle;
  float prev_phi;
  float drag_sensitive = 0.004f;
  int zoom_sensitive = 5;

  // light
  vec4 world_light;
};

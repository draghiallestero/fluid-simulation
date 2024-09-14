#ifndef GLOBAL_DATA_HPP
#define GLOBAL_DATA_HPP

#include "rk2_integrator.hpp"

#include "raylib.h"

#include <memory>
#include <vector>

struct Particle
{
  Vector2 pos_;
  Vector2 vel_;
  float radius_;
};

struct Wall
{
  Vector2 top_left_corner_;
  float width_;
  float height_;
  // float angle_;
};

struct GlobalData
{
  // Constants
  static constexpr int WINDOW_WIDTH = 1600; 
  static constexpr int WINDOW_HEIGHT = 900; 
  static constexpr int INITIAL_PARTICLE_COUNT = 1000; 
  static constexpr int SUBFRAMES = 4; 
  static constexpr float PARTICULE_RADIUS_MIN = 5.0;
  static constexpr float PARTICULE_RADIUS_MAX = 10.0;
  static constexpr float TARGET_FPS = 60.0;
  static constexpr float TARGET_DELTA_TIME = 1.0 / 60.0;
  static constexpr float WALL_WIDTH = 10;
  

  std::vector<std::unique_ptr<Particle>> particles_;
  std::vector<RK2Integrator> integrators_;
  std::vector<Wall> walls_;
};
extern GlobalData gd;

#endif // GLOBAL_DATA_HPP

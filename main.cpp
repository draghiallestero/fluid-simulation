#include "raylib.h"
#include "raymath.h"

#include "global_data.hpp"

#include <algorithm>
#include <cmath>
#include <vector>




#include <sstream>

void update_accelerations();

void AddParticle(Vector2 pos, Vector2 vel);

void AddIntegrator(Vector2 *pv, const Vector2 *pvd);

void CheckCollisionParticleParticle(Particle *pparticle, Particle *pother_particle);

void CheckCollisionParticleWall(Particle *pparticle, Wall *pwall);

int main(void)
{
  InitWindow(1600, 900, "Fluid Simulation");

  SetTargetFPS(60);

  SetRandomSeed(time(nullptr));

  // Randomly initialize particles
  for (int i = 0; i < GlobalData::INITIAL_PARTICLE_COUNT; ++i)
  {
    Vector2 pos{float(GetRandomValue(0, GlobalData::WINDOW_WIDTH)), float(GetRandomValue(0, GlobalData::WINDOW_HEIGHT))};
    Vector2 vel{float(GetRandomValue(-5, 5)), float(GetRandomValue(-5, 5))};
    AddParticle(pos, vel);
  }

  AddParticle({100, 620}, {0, 10});
  AddParticle({100, 600}, {0, 10});

  // Initialize walls
  // Wall left{{-GlobalData::WALL_WIDTH, 200}, GlobalData::WALL_WIDTH, 700};
  // gd.walls_.push_back(left);
  Wall bottom{{0, GlobalData::WINDOW_HEIGHT - 200}, GlobalData::WINDOW_WIDTH, GlobalData::WALL_WIDTH};
  gd.walls_.push_back(bottom);

  while (!WindowShouldClose())
  {
    BeginDrawing();
    ClearBackground(BLACK);
    DrawText("Congrats! You created your first window!", 190, 200, 20, MAGENTA);

    static bool go = false;
    if (IsKeyPressed(KEY_SPACE))
    {
      go = true;
    }
    if (!go)
    {
      goto end;
    }

    // Propagate states
    for (auto &integrator : gd.integrators_)
    {
      integrator.step1(GlobalData::TARGET_DELTA_TIME);
    }
    // Acceleration is not dependent on time so we there's no time variable
    for (auto &integrator : gd.integrators_)
    {
      integrator.step2(GlobalData::TARGET_DELTA_TIME);
    }
    for (auto &integrator : gd.integrators_)
    {
      integrator.apply();
    }

    // Spawn new particles
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
      AddParticle({float(GetMouseX()), float(GetMouseY())}, {0.0f, 0.0f});
    }

    // This process must be repeated a few times per frame to avoid jitters
    for (int count = 0; count < GlobalData::SUBFRAMES; ++count)
    {
      for (int i = 0; i < gd.particles_.size(); ++i)
      {
        Particle *pparticle = gd.particles_[i].get();

        // Check collisions against other particles
        for (int j = i + 1; j < gd.particles_.size(); ++j)
        {
          Particle *pother_particle = gd.particles_[j].get();
          CheckCollisionParticleParticle(pparticle, pother_particle);
        }

        // Check collisions against walls
        for (Wall &wall : gd.walls_)
        {
          CheckCollisionParticleWall(pparticle, &wall);
        }
      }
    }

    for (auto &pparticle : gd.particles_)
    {
      // DrawCircle(particle.pos_.x, particle)
      DrawCircleV(pparticle->pos_, pparticle->radius_, GRAY);
    }

    // Draw walls
    for (Wall &wall : gd.walls_)
    {
      DrawRectangle(wall.top_left_corner_.x, wall.top_left_corner_.y, wall.width_, wall.height_, PURPLE);
    }



end:




    EndDrawing();
  }

  CloseWindow();

  return 0;
}

void AddParticle(Vector2 pos, Vector2 vel)
{
  // Assign particle data
  auto pparticle = std::make_unique<Particle>();
  pparticle->pos_ = pos;
  pparticle->vel_ = vel;
  pparticle->radius_ = GetRandomValue(0, 100) * (GlobalData::PARTICULE_RADIUS_MAX - GlobalData::PARTICULE_RADIUS_MIN) / 100.0f + GlobalData::PARTICULE_RADIUS_MIN;
  // Make integrators
  static const Vector2 acc{0, 500};
  AddIntegrator(&pparticle->pos_, &pparticle->vel_);
  AddIntegrator(&pparticle->vel_, &acc);
  // Store particle
  gd.particles_.push_back(std::move(pparticle));
}

void AddIntegrator(Vector2 *pv, const Vector2 *pvd)
{
  gd.integrators_.push_back(RK2Integrator(&pv->x, &pvd->x));
  gd.integrators_.push_back(RK2Integrator(&pv->y, &pvd->y));
}

void CheckCollisionParticleParticle(Particle *pparticle, Particle *pother_particle)
{
  // The collision check does not require any calls to sqrt, but it requires
  // a bit of roundabout calculation
  Vector2 pos_to_other_pos = Vector2Subtract(pother_particle->pos_, pparticle->pos_);
  const float pos_to_other_pos_norm = Vector2LengthSqr(pos_to_other_pos);
  const float radii = pparticle->radius_ + pother_particle->radius_;
  const float radii_squared = radii * radii;
  if (pos_to_other_pos_norm < radii_squared)
  {
    // Move circles out of the way
    const float pos_to_other_pos_mag = std::sqrt(pos_to_other_pos_norm);
    const float overlap = radii - pos_to_other_pos_mag;
    const Vector2 pos_to_other_pos_unit = Vector2Scale(pos_to_other_pos, 1.0f / pos_to_other_pos_mag);
    const Vector2 other_pos_to_pos_unit = Vector2Negate(pos_to_other_pos_unit);
    pparticle->pos_       = Vector2Add(pparticle->pos_,       Vector2Scale(other_pos_to_pos_unit, overlap * pother_particle->radius_ / radii));
    pother_particle->pos_ = Vector2Add(pother_particle->pos_, Vector2Scale(pos_to_other_pos_unit, overlap * pparticle->radius_       / radii));

    // Calculate new velocities
    const float mass = pparticle->radius_* pparticle->radius_;
    const float other_mass = pother_particle->radius_* pother_particle->radius_;
    const Vector2 vel_to_other_vel = Vector2Subtract(pother_particle->vel_, pparticle->vel_);
    Vector2 vel_offset = Vector2Scale(pos_to_other_pos, 2.0f * other_mass / (mass + other_mass) * Vector2DotProduct(pos_to_other_pos, vel_to_other_vel) / pos_to_other_pos_norm);
    pparticle->vel_ = Vector2Add(pparticle->vel_, vel_offset);
    vel_offset = Vector2Negate(Vector2Scale(vel_offset, mass / other_mass));
    pother_particle->vel_ = Vector2Add(pother_particle->vel_, vel_offset);
  }
}

void CheckCollisionParticleWall(Particle *pparticle, Wall *pwall)
{
  // Figure out the closest point on the wall
  Vector2 on_wall = pparticle->pos_;

  // Clamp
  on_wall.x = std::min(std::max(on_wall.x, pwall->top_left_corner_.x), pwall->top_left_corner_.x + pwall->width_);
  on_wall.y = std::min(std::max(on_wall.y, pwall->top_left_corner_.y), pwall->top_left_corner_.y + pwall->height_);

  // Colliding in the x direction
  float pos_to_on_wall = on_wall.x - pparticle->pos_.x;
  if (std::fabs(pos_to_on_wall) < pparticle->radius_)
  {
    pparticle->pos_.x -= pos_to_on_wall;
    pparticle->vel_.x = 0.0f;
  }
  // Colliding in the y direction
  pos_to_on_wall = on_wall.y - pparticle->pos_.y;
  if (std::fabs(pos_to_on_wall) < pparticle->radius_)
  {
    pparticle->pos_.y -= std::copysign(pparticle->radius_ - std::fabs(pos_to_on_wall), pos_to_on_wall);
    pparticle->vel_.y = 0.0f;
  }
}
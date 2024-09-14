#ifndef RK2_INTEGRATOR_HPP
#define RK2_INTEGRATOR_HPP

// See https://www.ctcms.nist.gov/~langer/oof2man/RegisteredClass-RK2.html
struct RK2Integrator {
  // State
  float *px_;
  // State derivative
  const float *pxd_;
  // Saved value
  float xlast_;
  // Saved derivative value
  float xdlast_;
  // Next value
  float xnext_;

  // Return new integrator
  RK2Integrator(float *px, const float *pxd) {
    px_ = px;
    pxd_ = pxd;
  }

  // Update time
  static void update_time(float *pt, float dt) {
    *pt += dt / 2.0f;
  }

  // Calculate next state
  void step1(float dt) {
    xlast_ = *px_;
    const float k1 = dt * *pxd_;
    xnext_ = xlast_ + k1 / 2.0f;
  }

  // Calculate next state
  void step2(float dt) {
    const float k2 = dt * *pxd_;
    xnext_ = xlast_ + k2;
  }

  // Apply next state
  void apply() {
    *px_ = xnext_;
  }
};

#endif // RK2_INTEGRATOR_HPP

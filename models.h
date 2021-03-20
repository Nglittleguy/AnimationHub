#pragma once

#include <glm/glm.hpp>

#include <vector>
using namespace std;
namespace simulation {

using vec2f = glm::vec2;
using vec3f = glm::vec3;

struct Model {
  virtual ~Model() = default;
  virtual void reset() = 0;
  virtual void step(float dt) = 0;
};

struct Particle {
  explicit Particle(vec3f position) : x(position) {}
  Particle(vec3f position, vec3f velocity) : x(position), v(velocity) {}

  vec3f x;
  vec3f v = vec3f{0.f};
};

//
// Small angle pendulum
//
class SmallAnglePendulumModel : public Model {
public:
  SmallAnglePendulumModel();
  void reset() override;
  void step(float dt) override;

public:
  // constants
  float const gravity = 9.81f;
  float const theta0 = 0.5f;
  float const armLength = 5.f;
  float const mass = 1.f;

  // dependent variables
  float t = 0.f;
  float theta = theta0;
};
// free functions
float smallAnglePendulum(float t, float theta0, float l, float mass,
                         float graivty);

vec3f pendulumPosition(float theta, float l);

//
// Double Pendulum
//
class DoublePendulumModel : public Model {
public:
  DoublePendulumModel();

  void reset() override;
  void step(float dt) override;

  vec3f mass0Position() const;
  vec3f mass1Position() const;

public:
  // constants
  float const g = 9.81f;
  float const l = 10.f; // arm lengths
  float const m = 1.f;  // mass

  // dependent variables
  // angle
  float theta0 = 0.f;
  float theta1 = 0.f;

  // momentum
  float p0 = 0.f;
  float p1 = 0.f;
};

//
// Particle Model
//
class ParticleModel : public Model {
public:
  ParticleModel();
  void reset() override;
  void step(float dt) override;

public:
  std::vector<Particle> particles;
  float bounds = 10.f;
};

//
// Mass Spring 1
//
class MassSpring1Model : public Model {
public:
    MassSpring1Model();
    void reset() override;
    void step(float dt) override;
    vec3f mass0Position() const;
    vec3f mass1Position() const;

public:
    // constants
    float const gravity = 9.81f;
    float const d = -7.f;
    float const mass = 1.f;
    float const k = 30.f;
    float const airDamp = 1.f;

    // dependent variables
    float t = 0.f;
    float x = -7.f;
    float v = -20.f;
};

//
// Mass Spring 2
//
class MassSpring2Model : public Model {
public:
    MassSpring2Model();
    void reset() override;
    void step(float dt) override;
    vec3f massPosition(int i) const;

public:
    // constants
    float const gravity = 9.81f;
    float const d = 3.f;
    float const mass = 1.f;
    float const k = 30.f;
    float const airDamp = 0.2f;
    int const a = 6;

    // dependent variables
    float t = 0.f;
    std::vector<vec3f> x = { {0.f,0.f,0.f}, {3.0f, 0.f, 0.f}, {6.f,0.f,0.f}, {9.0f, 0.f, 0.f} , 
        {12.f,0.f,0.f}, {15.0f, 0.f, 0.f} , {18.f,0.f,0.f}, {21.0f, 0.f, 0.f}, {24.f,0.f,0.f}, {27.0f, 0.f, 0.f} };
    std::vector<vec3f> v = { {0.f,0.f,0.f}, {1.0f, 0.f, 0.f}, {0.f,0.f,0.f}, {0.0f, 0.f, 0.f} ,
        {0.f,0.f,0.f}, {0.0f, 0.f, 0.f} , {0.f,0.f,0.f}, {0.0f, 0.f, 0.f}, {0.f,0.f,0.f}, {0.0f, 0.f, 0.f} };

};


} // namespace simulation

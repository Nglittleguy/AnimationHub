#include "models.h"
#include "iostream"

namespace simulation {

//
// Small angle pendulum
//
SmallAnglePendulumModel::SmallAnglePendulumModel() { reset(); }

void SmallAnglePendulumModel::reset() {
  t = 0.f;
  theta = theta0;
}

void SmallAnglePendulumModel::step(float dt) {
  t += dt;
  theta = smallAnglePendulum(t, theta0, armLength, mass, gravity);
}

float smallAnglePendulum(float t, float theta0, float l, float mass,
                         float graivty) {
  using std::cos;
  using std::sqrt;
  return theta0 * cos(sqrt(graivty / l) * t);
}

vec3f pendulumPosition(float theta, float l) {
  //   /
  //	/
  // o
  using std::cos;
  using std::sin;

  float y = -l * cos(theta);
  float x = l * sin(theta);
  return {x, y, 0.f};
}

//
// Double Pendulum
//
DoublePendulumModel::DoublePendulumModel() { reset(); }

void DoublePendulumModel::reset() {
  theta0 = 5.f;
  theta1 = 10.f;
  p0 = 0.f;
  p1 = 0.f;
}

void DoublePendulumModel::step(float dt) {

  float cosDeltaTheta = std::cos(theta0 - theta1);
  float sinDeltaTheta = std::sin(theta0 - theta1);
  float denom = (m * l * l) * (16.f - 9.f * cosDeltaTheta * cosDeltaTheta);

  // velocities
  float v0 = 6.f * (2.f * p0 - 3.f * cosDeltaTheta * p1) / denom;
  float v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;

  // forces
  float f0 = -(0.5f * m * l * l) *
             (v0 * v1 * sinDeltaTheta + 3.f * (g / l) * std::sin(theta0));
  float f1 = -(0.5f * m * l * l) *
             (-v0 * v1 * sinDeltaTheta + (g / l) * std::sin(theta1));

  // update kinematic/dynamic quantites using Euler integration
  // update momentum
  p0 = p0 + f0 * dt;
  p1 = p1 + f1 * dt;

  // Semi-implicit Euler
  // would use the updated momemnta/velocities for the position updates
  v0 = 6.f * (2.f * p0 - 3.f * cosDeltaTheta * p1) / denom;
  v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;

  // update (angular) positions
  theta0 = theta0 + v0 * dt;
  theta1 = theta1 + v1 * dt;
}

vec3f DoublePendulumModel::mass0Position() const {
  return l * vec3f(std::sin(theta0), -std::cos(theta0), 0.f);
}

vec3f DoublePendulumModel::mass1Position() const {
  using std::cos;
  using std::sin;
  return l * vec3f(sin(theta0) + sin(theta1), -cos(theta0) - cos(theta1), 0.f);
}

//
// Particle Model
//
ParticleModel::ParticleModel() { reset(); }

void ParticleModel::reset() {
  particles.clear();
  // setup
  for (int i = -5; i <= 5; ++i) {
    particles.push_back(Particle({i, i, 0.f}, {-i, 3.f * i, 0.f}));
  }
}

void ParticleModel::step(float dt) {
  for (int iter = 0; iter < 16; ++iter) {
    // do collisions
    for (auto &p : particles) {
      if (length(p.x) > bounds) {
        auto n = normalize(p.x);
        p.v = glm::reflect(p.v, n);
      }
    }

    // move particles
    for (auto &p : particles) {
      // forward Euler
      p.x += p.v * dt;
    }
  }
}

//
// Mass Spring 1
//
MassSpring1Model::MassSpring1Model() { reset(); }

void MassSpring1Model::reset() {
    t = 0.f;
    x = -7.f;
}

void MassSpring1Model::step(float dt) {
    

    // velocities

    // forces
    float fS = k * (d-x) / mass;
    float fG = mass * gravity;
    float fA = 0.f;
    if (fS < 0)
        fA = airDamp*v;
    else if (fS>0)
        fA = -airDamp*v;

    float fTotal = fS - fG + fA;
    
    //float f1 = -(0.5f * m * l * l) *
    //    (-v0 * v1 * sinDeltaTheta + (g / l) * std::sin(theta1));

    // update kinematic/dynamic quantites using Euler integration
    
    // Semi-implicit Euler
    // would use the updated momemnta/velocities for the position updates
    v += dt * fTotal / mass;

    //v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;


    // update positions
    x += dt * v;
}

vec3f MassSpring1Model::mass0Position() const {
    return vec3f{ 0.f, 0.f, 0.f };
}

vec3f MassSpring1Model::mass1Position() const {
    return vec3f{ 0.f, x, 0.f };
}


//
// Mass Spring 2
//
MassSpring2Model::MassSpring2Model() { reset(); }

void MassSpring2Model::reset() {
    t = 0.f;
    x = { {0.f,0.f,0.f}, {3.0f, 0.f, 0.f}, {6.f,0.f,0.f}, {9.0f, 0.f, 0.f} ,
        {12.f,0.f,0.f}, {15.0f, 0.f, 0.f} , {18.f,0.f,0.f}, {21.0f, 0.f, 0.f}, {24.f,0.f,0.f}, {27.0f, 0.f, 0.f} };
    v = { {0.f,0.f,0.f}, {0.0f, 0.f, 0.f}, {0.f,0.f,0.f}, {0.0f, 0.f, 0.f} ,
        {0.f,0.f,0.f}, {0.0f, 0.f, 0.f} , {0.f,0.f,0.f}, {0.0f, 0.f, 0.f}, {0.f,0.f,0.f}, {0.0f, 0.f, 0.f} };
}

void MassSpring2Model::step(float dt) {


    // velocities

    // forces
    vector<vec3f> fS;
    for (int i = 0; i < a; i++) {
        fS.push_back(vec3f{ 0.f,0.f,0.f });
    }
    for (int i = 0; i < a-1; i++) {
        
        vec3f force = -k * (length(x[i+1] - x[i]) - d) * normalize(x[i+1] - x[i]);
        fS[i+1] += force;
        fS[i] -= force;
        //std::cout << "(" << fS[1][0] << "," << fS[1][1] << "," << fS[1][2] << ")" << std::endl;
        //fS[i + 1] -= force;
    }

    vec3f fG = { 0,-mass * gravity, 0 };
    //vector<vec3f> fA;
    //for (int i = 1; i < 10; i++) {
    //    fA.push_back(-airDamp * v[i]);
    //}

    vector<vec3f> fTotal;
    fTotal.push_back( vec3f{0.f,0.f,0.f} );
    for (int i = 1; i < a; i++) {
        fTotal.push_back(fG +fS[i] + -airDamp*v[i]);
    }

    // update kinematic/dynamic quantites using Euler integration

    // Semi-implicit Euler
    // would use the updated momemnta/velocities for the position updates
    for (int i = 1; i < a; i++) {
        v[i] += dt * fTotal[i] / mass;
        x[i] += dt * v[i];
    }
  
}

vec3f MassSpring2Model::massPosition(int i) const {
    if (i == 0)
        return vec3f{ 0.f, 0.f, 0.f };
    else
        return x[i];
}



} // namespace simulation

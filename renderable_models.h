#pragma once

#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "models.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

namespace simulation {
//
// Mass Spring Model 1
//
template <typename View>
void render(MassSpring1Model const &model, View const &view) {

  auto mass_geometry = Sphere(Radius(1.f));
  auto mass_style = Phong(Colour(1.f, 0.f, 1.f), LightPosition(100.f, 100.f, 100.f));
  static auto mass_renderable = createInstancedRenderable(mass_geometry, mass_style);

  //auto point = pendulumPosition(model.theta, model.armLength);
  auto m0 = model.mass0Position();
  auto m1 = model.mass1Position();

  auto M = translate(mat4f{1.f}, m1);
  addInstance(mass_renderable, M);
  M = translate(mat4f{ 1.f }, m0);
  addInstance(mass_renderable, M);

  static auto arm_renderable = createRenderable(
      Line(Point1(0.f, 0.f, 0.f), Point2(0.f, -1.f, 0.f)), // geometry
      LineStyle(Colour(1.f, 0.f, 1.f))                     // style
  );

  auto arm_geometry = Line(Point1(m0), Point2(m1));

  updateRenderable(arm_geometry,                     // new position
                   LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
                   arm_renderable);

  draw(arm_renderable, view);
  draw(mass_renderable, view);
}

//
// Mass Spring Model 2
//
template <typename View>
void render(MassSpring2Model const& model, View const& view) {

    auto mass_geometry = Sphere(Radius(1.f));
    auto mass_style = Phong(Colour(1.f, 0.f, 1.f), LightPosition(100.f, 100.f, 100.f));
    static auto mass_renderable = createInstancedRenderable(mass_geometry, mass_style);

    //auto point = pendulumPosition(model.theta, model.armLength);
    vector<vec3f> m;
    for (int i = 0; i < 6; i++) {
        vec3f mi = model.massPosition(i);
        auto M = translate(mat4f{ 1.f }, mi);
        addInstance(mass_renderable, M);
        m.push_back(mi);
    }

    static auto arm_renderable =
        createRenderable(PolyLine<PrimitiveType::LINE_STRIP>(), // geometry
            LineStyle(Colour(1.f, 1.f, 0.f))       // style
        );

    auto arm_geometry = PolyLine<PrimitiveType::LINE_STRIP>(Point(m[0]), Point(m[1]), Point(m[2]), Point(m[3]),
        Point(m[4]), Point(m[5]));// , Point(m[6]), Point(m[7]), Point(m[8]), Point(m[9]));

    updateRenderable(arm_geometry,                     // new position
        LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
        arm_renderable);

    draw(arm_renderable, view);
    draw(mass_renderable, view);
}


//
// Pendulum model
//
template <typename View>
void render(SmallAnglePendulumModel const &model, View const &view) {

  auto mass_geometry = Sphere(Radius(1.f));
  auto mass_style = Phong(Colour(1.f, 0.f, 1.f), //
                          LightPosition(100.f, 100.f, 100.f));
  static auto mass_renderable =
      createInstancedRenderable(mass_geometry, mass_style);

  auto point = pendulumPosition(model.theta, model.armLength);

  auto M = translate(mat4f{1.f}, point);
  addInstance(mass_renderable, M);

  static auto arm_renderable = createRenderable(
      Line(Point1(0.f, 0.f, 0.f), Point2(0.f, -1.f, 0.f)), // geometry
      LineStyle(Colour(1.f, 0.f, 1.f))                     // style
  );

  auto arm_geometry = Line(Point1(0.f, 0.f, 0.f), Point2(point));

  updateRenderable(arm_geometry,                     // new position
                   LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
                   arm_renderable);

  draw(arm_renderable, view);
  draw(mass_renderable, view);
}

//
// Double pendulum Model
//
template <typename View>
void render(DoublePendulumModel const &model, View const &view) {

  auto mass_geometry = Sphere(Radius(1.f));
  auto mass_style = Phong(Colour(1.f, 0.f, 1.f), //
                          LightPosition(100.f, 100.f, 100.f));
  static auto mass_renderable =
      createInstancedRenderable(mass_geometry, mass_style);

  auto m0 = model.mass0Position();
  auto m1 = model.mass1Position();

  auto M = translate(mat4f{1.f}, m0);
  addInstance(mass_renderable, M);
  M = translate(mat4f{1.f}, m1);
  addInstance(mass_renderable, M);

  static auto arm_renderable =
      createRenderable(PolyLine<PrimitiveType::LINE_STRIP>(), // geometry
                       LineStyle(Colour(1.f, 1.f, 0.f))       // style
      );

  auto arm_geometry = PolyLine<PrimitiveType::LINE_STRIP>(Point(0.f, 0.f, 0.f),
                                                          Point(m0), Point(m1));

  updateRenderable(arm_geometry,                     // new position
                   LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
                   arm_renderable);

  draw(arm_renderable, view);
  draw(mass_renderable, view);
}

//
// Particle Model
//
template <typename View>
void render(ParticleModel const &model, View const &view) {
  // static: only initiallized once, the first time this function is called.
  // This isn't the most elegant method to do this but it works. Just put your
  // draw calls in here.

  static auto point_renders = createInstancedRenderable(
      Sphere(Radius(0.25f)), // geometry
      Phong(Colour(1.f, 0.4f, 0.2f),
            LightPosition(100.f, 100.f, 100.f)) // style
  );

  // load up renderable
  auto const &particles = model.particles;

  for (auto const &particle : particles) {
    auto M = translate(mat4f{1.f}, particle.x);
    addInstance(point_renders, M);
  }

  // draw the renderable
  draw(point_renders, view);
}

//
// Helper class/functions
//
template <typename View> struct RenderableModel {

  template <typename Model>
  RenderableModel(Model const &model) : m_self(new model_t<Model>(model)) {}

  friend void render(RenderableModel const &renderable, View const &view) {
    renderable.m_self->renderSelf(view);
  }

  struct concept_t {
    virtual ~concept_t() = default;
    virtual void renderSelf(View const &view) const = 0;
  };

  template <typename Model> struct model_t : public concept_t {
    model_t(Model const &model) : data(model) {}
    void renderSelf(View const &view) const override { render(data, view); }
    Model const &data;
  };

  std::shared_ptr<concept_t const> m_self;
};

template <typename Model, typename View>
RenderableModel<View> makeModelRenderable(Model const &model,
                                          View const &view) {
  return RenderableModel<View>(model);
}

} // namespace simulation

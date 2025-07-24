#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  // const auto b = dot(r.d,(r.o - this->o));
  // const auto b2 = pow(b,2);
  //
  // const auto four_ac = dot(r.d, r.d) * (dot(r.o - this->o,r.o - this->o) - this->r2);
  // auto b2_4ac = b2 - four_ac;
  // if (b2_4ac < 0) { //cant square root a negative (discriminant)
  //   return false;
  // }
  //
  // b2_4ac = pow(b2_4ac, .5);
  //
  // auto t_pos = (-b + b2_4ac)/dot(r.d, r.d);
  // auto t_neg = (-b - b2_4ac)/dot(r.d,r.d);
  //
  // if (dot(r.d, r.d)*pow(t_pos,2) + dot(2*r.d,(r.o - this->o)*t_pos) + dot(r.o - this->o,r.o - this->o) - r2== 0) {
  //   return true;
  // }
  //
  // if (dot(r.d, r.d)*pow(t_neg,2) + dot(2*r.d,(r.o - this->o)*t_neg) + dot(r.o - this->o,r.o - this->o) - r2== 0) {
  //   return true;
  // }

  auto a = dot(r.d,r.d);
  auto b = dot(2*(r.o - this->o),r.d);
  auto c = dot(r.o - this->o,r.o - this->o) - r2 ;
  if (pow(b,2) - 4*a*c < 0) {
    return false;
  }
  auto t_pos =(-b + pow(pow(b,2) - 4*a*c,.5))/(2*a);
  auto t_neg = (-b - pow(pow(b,2) - 4*a*c,.5))/(2*a);

  auto min_t = min(t_neg,t_pos);
  auto max_t = max(t_neg,t_pos);
  if (r.max_t >= min_t && r.min_t <= min_t) {
    return true;
  }
  if (r.max_t >= max_t && r.min_t <= max_t) {
    return true;
  }
  return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  auto a = dot(r.d,r.d);
  auto b = dot(2*(r.o - this->o),r.d);
  auto c = dot(r.o - this->o,r.o - this->o)  - r2 ;

  if (pow(b,2) - 4*a*c < 0) {
    return false;
  }

  auto t_pos =(-b + pow(pow(b,2) - 4*a*c,.5))/(2*a);
  auto t_neg = (-b - pow(pow(b,2) - 4*a*c,.5))/(2*a);

  auto min_t1 = min(t_neg,t_pos);
  auto max_t1 = max(t_neg,t_pos);
  if (r.max_t >= min_t1 && r.min_t <= min_t1) {
    r.max_t = max_t1;
    r.min_t = min_t1;
    i->t = min_t1;
    i->bsdf = this->get_bsdf();
    i->primitive = this;
    auto p = r.o + min_t1*r.d;
    i->n = (p - this->o).unit();
    // i->n = i->n.unit();
    return true;
  }
  if (r.max_t >= max_t1 && r.min_t <= max_t1) {
    r.max_t = max_t1;
    r.min_t = min_t1;
    i->t = max_t1;
    i->bsdf = this->get_bsdf();
    i->primitive = this;
    auto p = r.o + max_t1*r.d;
    i->n = (p - this->o).unit();
    return true;
  }
  return false;

  // const auto b = dot(r.d,r.o - this->o);
  // const auto b2 = pow(b,2);
  //
  // const auto four_ac = dot(r.d, r.d) * (dot(r.o - this->o,r.o - this->o) - this->r2);
  // auto b2_4ac = b2 - four_ac;
  // if (b2_4ac < 0) { //cant square root a negative (discriminant)
  //   return false;
  // }
  //
  // b2_4ac = pow(b2_4ac, .5);
  //
  // auto t_pos = (-b + b2_4ac)/dot(r.d, r.d);
  // auto t_neg = (-b - b2_4ac)/dot(r.d,r.d);
  //
  // // cout<<dot(r.d, r.d)*pow(t_pos,2) + dot(2*r.d,(r.o - this->o)*t_pos) + dot(r.o - this->o,r.o - this->o) - r2<<endl;
  //
  // if (t_neg >= r.min_t && t_neg <= r.max_t) {
  //   // cout<<"s"<<endl;
  //   Vector3D norm = 2*(r.o + t_neg*r.d - o);
  //   i->t = t_neg;
  //   i->n = norm;
  //   i->bsdf = this->get_bsdf();
  //   i->primitive = this;
  //   return true;
  // }


  // if (dot(r.d, r.d)*pow(t_neg,2) + dot(2*r.d,(r.o - this->o)*t_neg) + dot(r.o - this->o,r.o - this->o) - r2== 0) {
  //
  // }

  // if (dot(r.d, r.d)*pow(t_pos,2) + dot(2*r.d,(r.o - this->o)*t_pos) + dot(r.o - this->o,r.o - this->o) - r2 == 0) {
  //   if (t_pos >= r.min_t && t_pos <= r.max_t) {
  //     cout<<"s"<<endl;
  //     Vector3D norm = (2*(r.o + t_pos*r.d - o))/this->r;
  //     i->t = t_pos;
  //     i->n = norm;
  //     i->bsdf = this->get_bsdf();
  //     i->primitive = this;
  //     return true;
  //   }
  //
  // }
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL

#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"
#include "vector3D.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  // Algorithim Derived from Wikipedia: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

  Vector3D e1 = p2 - p1;
  Vector3D e2 = p3 - p1;
  Vector3D ray_cross_e2 = cross(r.d, e2);
  double det = dot(e1, ray_cross_e2);
  if (det < 1e-6 && det > -1e-6) {
    return false; // Ray is parallel to the triangle
  }
  Vector3D s = r.o - p1;   //distance from p1 to ray origin
  double u = (1.0/det) * dot(s, ray_cross_e2);   //u parameter

  Vector3D q = cross(s, e1); //vector perpendicular to both s and edge1
  double v = (1.0/det) * dot(r.d, q);

  double w = 1 - u - v;

  if(u>1.0 || v>1.0 || u<0.0 || v<0.0 || w<0.0 || w>1.0) {
    return false;
  }

  double t = (1.0/det) * dot(e2, q);
  if (t >= r.min_t && t <= r.max_t) {  // Check if intersection is within valid t range
    return true;
  }else{ return false;}
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D e1 = p2 - p1;
  Vector3D e2 = p3 - p1;
  Vector3D ray_cross_e2 = cross(r.d, e2);
  double det = dot(e1, ray_cross_e2);
  if (det < 1e-6 && det > -1e-6) {
    return false; // Ray is parallel to the triangle
  }
  Vector3D s = r.o - p1;   //distance from p1 to ray origin
  double u = (1.0/det) * dot(s, ray_cross_e2);   //u parameter

  Vector3D q = cross(s, e1); //vector perpendicular to both s and edge1
  double v = (1.0/det) * dot(r.d, q);

  double w = 1.0 - u - v;

  if(u>1.0 || v>1.0 || u<0.0 || v<0.0 || w<0.0 || w>1.0) {
    return false;
  }

  double t = (1.0/det) * dot(e2, q);
  if (t >= r.min_t && t <= r.max_t) {
    // interpolate normal using barycentric coordinates with vertex normals
    Vector3D interpolated_normal = (w * n1) + (u * n2) + (v * n3);
    interpolated_normal = interpolated_normal.unit();
    
    // populate its input Intersection *isect structure
    isect->t = t;
    isect->n = interpolated_normal;
    isect->bsdf = bsdf;
    isect->primitive = this;    

    return true;
  }else{return false;}

}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL

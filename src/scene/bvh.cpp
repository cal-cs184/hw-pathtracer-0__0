#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  

  BBox bbox;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);
  node->start = start;
  node->end = end;

  size_t num = end - start;
  if (num <= max_leaf_size) {
    // If the number of primitives is less than or equal to the maximum
    // leaf size, we create a leaf node.
    node->l = nullptr;
    node->r = nullptr;
    return node;
  }

  BBox centroid_bounds;
  for (auto i = start; i != end; i++) {
    Vector3D c = (*i)->get_bbox().centroid();
    centroid_bounds.expand(c);  
  }

  double x_len = centroid_bounds.max.x - centroid_bounds.min.x;
  double y_len = centroid_bounds.max.y - centroid_bounds.min.y;
  double z_len = centroid_bounds.max.z - centroid_bounds.min.z;

  int axis = 0;
  if (y_len > x_len && y_len > z_len) {
    axis = 1; // Y-axis
  } else if (z_len > x_len && z_len > y_len) {
    axis = 2; // Z-axis
  } // else axis remains 0 (X-axis)

  double mid = (centroid_bounds.max[axis] + centroid_bounds.min[axis]) / 2.0;

  std::vector<Primitive *> left_primitives;
  std::vector<Primitive *> right_primitives;

  for (auto i = start; i != end; i++) {
    double center = (*i)->get_bbox().centroid()[axis];
    if (center <= mid) left_primitives.push_back(*i);
    else right_primitives.push_back(*i);
  }



  return node;


}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.



  for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
      return true;
  }
  return false;


}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.



  bool hit = false;
  for (auto p : primitives) {
    total_isects++;
    hit = p->intersect(ray, i) || hit;
  }
  return hit;


}

} // namespace SceneObjects
} // namespace CGL

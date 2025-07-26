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

void axis_split(std::vector<Primitive *> *first_vector,
                        std::vector<Primitive *> *second_vector,
                        BBox *first_box,
                        BBox *second_box,
                        std::vector<Primitive *>::iterator start,
                        std::vector<Primitive *>::iterator end, BVHNode *node, int axis) {

  for (auto curr = start; curr != end; ++curr) {
    const auto prim = *curr;
    bool prim_axis;

    if (axis == 0) {prim_axis = prim->get_bbox().centroid().x <= node->bb.centroid().x;}
    else if (axis == 1) {prim_axis = prim->get_bbox().centroid().y <= node->bb.centroid().y;}
    else if (axis == 2) {prim_axis = prim->get_bbox().centroid().z <= node->bb.centroid().z;}
    else{return;}

    if (prim_axis) {
      first_box->expand(prim->get_bbox());
      first_vector->push_back(prim);
    } else {
      second_box->expand(prim->get_bbox());
      second_vector->push_back(prim);
    }
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

  for (auto p = start; p != end ; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);

  long dist = std::distance(start, end);

  if (dist <= max_leaf_size) {
    node->l = NULL;
    node->r = NULL;

    node->start = start;
    node->end = end;

    return node;
  }

  // node->l = construct_bvh(start, end - (end-start)/2, max_leaf_size);
  // node->r = construct_bvh(end - (end-start)/2,end, max_leaf_size);
  // return node;

  // TODO: Refactor on x, y, and z
  BBox left_box;
  BBox right_box;

  std::vector<Primitive *> left_vector;
  std::vector<Primitive *> right_vector;

  axis_split(&left_vector,&right_vector,&left_box,&right_box, start, end, node,0);

  BBox above_box;
  BBox bellow_box;

  std::vector<Primitive *> above_vector;
  std::vector<Primitive *> bellow_vector;

  axis_split(&above_vector,&bellow_vector,&above_box,&bellow_box, start, end, node,1);

  BBox forward_box;
  BBox backward_box;

  std::vector<Primitive *> forward_vector;
  std::vector<Primitive *> backward_vector;

  axis_split(&forward_vector,&backward_vector,&forward_box,&backward_box, start, end, node,2);


  int first_diff = left_vector.size() - right_vector.size();
  int second_diff = above_vector.size() - bellow_vector.size();
  int third_diff = forward_vector.size() - backward_vector.size();
  int best_diff = min({first_diff, second_diff, third_diff});

  if (best_diff == first_diff) {
    node->l = construct_bvh(left_vector.begin(), left_vector.end(), max_leaf_size);
    node->r = construct_bvh(right_vector.begin(), right_vector.end(), max_leaf_size);
  } else if (best_diff == second_diff) {
    node->l = construct_bvh(above_vector.begin(), above_vector.end(), max_leaf_size);
    node->r = construct_bvh(bellow_vector.begin(), bellow_vector.end(), max_leaf_size);
  } else {
    node->l = construct_bvh(forward_vector.begin(), forward_vector.end(), max_leaf_size);
    node->r = construct_bvh(backward_vector.begin(), backward_vector.end(), max_leaf_size);
  }

  node->start = start;
  node->end = end;

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

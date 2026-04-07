#pragma once

#include "mowgli_brv_planner/types.hpp"

namespace brv
{

// Point-in-polygon test (ray casting).
bool point_in_polygon(const Point2D& pt, const Polygon2D& poly);

// Signed area of polygon (positive = CCW).
double polygon_area(const Polygon2D& poly);

// Compute convex hull (Andrew's monotone chain).
Polygon2D convex_hull(const Polygon2D& pts);

// Offset (shrink) a convex-ish polygon inward by distance.
// Simple parallel-edge offset — good enough for headland generation.
Polygon2D offset_polygon_inward(const Polygon2D& poly, double dist);

// Sample points along polygon edges at regular intervals.
std::vector<Point2D> sample_polygon_edges(const Polygon2D& poly, double spacing);

// Rotate point around origin by angle (radians).
Point2D rotate_point(const Point2D& pt, double angle);

// Unrotate (rotate by -angle).
Point2D unrotate_point(const Point2D& pt, double angle);

// Total path length.
double path_length(const Path2D& path);

// Line segment intersection test.
bool segments_intersect(const Point2D& a1, const Point2D& a2, const Point2D& b1, const Point2D& b2);

// Test if line segment is fully inside polygon (no crossings with edges).
bool segment_inside_polygon(const Point2D& a, const Point2D& b, const Polygon2D& poly);

}  // namespace brv

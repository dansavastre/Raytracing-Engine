#pragma once
#include "scene.h"

struct HitInfo {
    glm::vec3 normal;
    Material material;
    glm::vec3 barycentric;
};


bool intersectRayWithPlane(const Plane& plane, Ray& ray);

// Returns true if the point p is inside the triangle spanned by v0, v1, v2 with normal n.
bool pointInTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, const glm::vec3& n, const glm::vec3& p, HitInfo& hitInfo);

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2);

bool intersectRayWithTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, Ray& ray, HitInfo& hitInfo);
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo);
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray);
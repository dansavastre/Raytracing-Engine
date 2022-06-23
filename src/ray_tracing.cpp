#include "ray_tracing.h"
#include "shading.h"
#include "draw.h"
#include "draw.h"
#include "bounding_volume_hierarchy.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>



bool pointInTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, const glm::vec3& n, const glm::vec3& p, HitInfo& hitInfo)
{
    glm::vec3 ab = glm::cross(n, (v1.position - v0.position));
    ab /= glm::dot((v2.position - v0.position), ab);
    glm::vec3 ac = glm::cross(n, (v0.position - v2.position));
    ac /= glm::dot((v1.position - v0.position), ac);
    
    float gamma = glm::dot((p - v2.position), ac);
    float beta = glm::dot((p - v1.position), ab);
    float alpha = 1 - (beta + gamma);

    if (alpha < 0.0f || beta < 0.0f || gamma < 0.0f ||
        alpha > 1.0f || beta > 1.0f || gamma > 1.0f) {
        return false;
    }

    hitInfo.barycentric = {alpha, gamma, beta};


    return true;
}



bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    // The ray is parallel to the plane and orthogonal to the plane normal
    if (glm::dot(plane.normal, ray.direction) == 0.0f) {
        return false;
    }
    // calculate new t
    float t = (plane.D - glm::dot(ray.origin, plane.normal)) / glm::dot(plane.normal, ray.direction);
    
    // Check if the intersection point is behind the ray origin
    if (t <= 0.0f || ray.t <= t) {
        return false;
    }
    // only update the hitInfo if the new t is closer than the old one
    if (t < ray.t) {
        ray.t = t;
    }
    return true;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    plane.normal = glm::normalize(glm::cross(v0 - v2, v1 - v2));
    plane.D = glm::dot(plane.normal, v0);
    return plane;
}

bool intersectRayWithTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, Ray& ray, HitInfo& hitInfo)
{
    // create the plane from the 3 vertices
    Plane plane = trianglePlane(v0.position, v1.position, v2.position);
    float initialT = ray.t;
    
    glm::vec3 oldNormal = hitInfo.normal;
    Material oldMaterial = hitInfo.material;

    if (intersectRayWithPlane(plane, ray)) {
        // Check if the intersection point is inside the triangle
        glm::vec3 p = ray.origin + ray.direction * ray.t;

        if (pointInTriangle(v0, v1, v2, plane.normal, p, hitInfo)) {
            hitInfo.normal = hitInfo.barycentric.x * v0.normal + hitInfo.barycentric.y * v1.normal + hitInfo.barycentric.z * v2.normal;
            //hitInfo.normal = plane.normal;
            return true;
        } 
        else {
            // If the intersection point is outside the triangle, keep the old hitInfo
            ray.t = initialT;
            hitInfo.normal = oldNormal;
            hitInfo.material = oldMaterial;
            return false;
        }
    }

    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
     glm::vec3 center = ray.origin - sphere.center;
     float a = pow(ray.direction.x,2) + pow(ray.direction.y,2) + pow(ray.direction.z,2);
     float b = 2 * (ray.direction.x*center.x + ray.direction.y*center.y + ray.direction.z*center.z);
     float c = pow(center.x,2) + pow(center.y,2) + pow(center.z,2) - pow(sphere.radius,2);

     float t0 = (-b + sqrt(pow(b,2) - 4*a*c)) / (2*a);
     float t1 = (-b - sqrt(pow(b,2) - 4*a*c)) / (2*a);

     float discriminant = pow(b, 2) - 4 * a * c;
     if (discriminant < 0.0f) {
         return false;
     }
     else {
         if (t0 > 0.0f && t1 > 0.0f && glm::min(t0, t1) < ray.t) {
             ray.t = glm::min(t0, t1);
         }
         else if (t0 > 0.0f && t1 <= 0.0f && t0 < ray.t) {
             ray.t = t0;
         }
         else {
             ray.t = t1;
         }

         glm::vec3 intersection = ray.origin + ray.t * ray.direction;
         hitInfo.normal = glm::normalize(intersection - sphere.center);
         hitInfo.material = sphere.material;
         return true;
     }
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
     float xmin = box.lower.x;
     float ymin = box.lower.y;
     float zmin = box.lower.z;

     float xmax = box.upper.x;
     float ymax = box.upper.y;
     float zmax = box.upper.z;

     float txmin = (xmin - ray.origin.x) / ray.direction.x;
     float tymin = (ymin - ray.origin.y) / ray.direction.y;
     float tzmin = (zmin - ray.origin.z) / ray.direction.z;
    
     float txmax = (xmax - ray.origin.x) / ray.direction.x;
     float tymax = (ymax - ray.origin.y) / ray.direction.y;
     float tzmax = (zmax - ray.origin.z) / ray.direction.z;   
    
     float tinx = std::min(txmin, txmax);
     float toutx = std::max(txmin, txmax);

     float tiny = std::min(tymin, tymax);
     float touty = std::max(tymin, tymax);

     float tinz = std::min(tzmin, tzmax);
     float toutz = std::max(tzmin, tzmax);

     float tin = std::max(std::max(tinx, tiny), tinz);
     float tout = std::min(std::min(toutx, touty), toutz);

     if (tin < tout && ray.t > tin && tin > 0) {
         ray.t = tin;
     }

     return tin <= tout;
}

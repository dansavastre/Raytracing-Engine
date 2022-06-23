#pragma once
#include "scene.h"
#include "ray_tracing.h"
#include "bounding_volume_hierarchy.h"

glm::vec3 calculateColor(Scene scene, Ray ray, HitInfo hitInfo, BoundingVolumeHierarchy bvh);
glm::vec3 calculateSpecular(HitInfo hitInfo, PointLight pointLight, Ray ray);
glm::vec3 calculateDiffuse(HitInfo hitInfo, PointLight pointLight, Ray ray);
glm::vec3 clamp(glm::vec3& v);
bool pointInTheLight(PointLight pointLight, Ray ray, HitInfo hitInfo, BoundingVolumeHierarchy bvh);

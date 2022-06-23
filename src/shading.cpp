#include "shading.h"
#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "ray_tracing.h"
#include "screen.h"
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
#include <cmath>
#include <iostream>
#include <limits>

#define inf 3.40282e+38


glm::vec3 calculateSpecular(HitInfo hitInfo, PointLight pointLight, Ray ray) {

    glm::vec3 normal = glm::normalize(hitInfo.normal);
    glm::vec3 vertex = ray.origin + ray.direction * ray.t;
    glm::vec3 light = glm::normalize(vertex- pointLight.position);
    glm::vec3 reflect = glm::reflect(light,normal);
    glm::vec3 camera = glm::normalize(ray.origin-vertex);
    glm::vec3 phongSpecular = hitInfo.material.ks * pow(glm::max(glm::dot(reflect, camera),0.0f), hitInfo.material.shininess) * pointLight.color;
    phongSpecular = glm::clamp(phongSpecular, 0.0f, 1.0f);
    return phongSpecular;
}

glm::vec3 calculateDiffuse(HitInfo hitInfo, PointLight pointLight, Ray ray) {

    glm::vec3 normal = glm::normalize(hitInfo.normal);
    glm::vec3 vertex = ray.origin + ray.direction * ray.t;
    glm::vec3 light = glm::normalize(pointLight.position - vertex);
    glm::vec3 diffuse = hitInfo.material.kd * glm::max(glm::dot(normal, light), 0.0f) * pointLight.color;
    diffuse = glm::clamp(diffuse, 0.0f,1.0f);
    return diffuse;
}

glm::vec3 calculateColor(Scene scene, Ray ray, HitInfo hitInfo, BoundingVolumeHierarchy bvh){
    glm::vec3 color = glm::vec3(0.0f, 0.0f, 0.0f);

    for (const auto& light : scene.lights) {
        if (std::holds_alternative<PointLight>(light)) {
            // TODO: Add HardShadow check
            // Trace a ray from hit point to light and check for other objects in the way
            // If there is an object then light does not contribute to the shading of hitPoint
            
            const PointLight pointLight = std::get<PointLight>(light);
            glm::vec3 specular = calculateSpecular(hitInfo, pointLight, ray);
            glm::vec3 diffuse = calculateDiffuse(hitInfo, pointLight, ray);

            if (pointInTheLight(pointLight, ray, hitInfo, bvh)) {
                color += specular;
                color += diffuse;
            }
        } 
        else if (std::holds_alternative<SegmentLight>(light)) {
            const SegmentLight segmentLight = std::get<SegmentLight>(light);
            glm::vec3 vertex = ray.origin + ray.direction * ray.t;
            float segments = 50;
            for (int i = 0; i <= segments; i++) {
                Ray lightRay;
                lightRay.origin = segmentLight.endpoint0 * float(i / segments) + segmentLight.endpoint1 * float((segments - i) / segments);
                lightRay.direction = vertex - lightRay.origin;
                HitInfo info;
                std::vector<float> arr;
                if (bvh.intersect(lightRay, info)) {
                    glm::vec3 rayColor = segmentLight.color0 * float(i / segments) + segmentLight.color1 * float((segments - i) / segments);
                    PointLight pl = { lightRay.origin, rayColor };
                    glm::vec3 specular = 1 / segments * calculateSpecular(hitInfo, pl, ray);
                    glm::vec3 diffuse = 1 / segments * calculateDiffuse(hitInfo, pl, ray);
                    if (pointInTheLight(pl, ray, hitInfo, bvh)) {
                        color += specular;
                        color += diffuse;
                    }
                }
            }
        }
        else if (std::holds_alternative<ParallelogramLight>(light)) {
            const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
            // Perform your calculations for a parallelogram light.
            glm::vec3 vertex = ray.origin + ray.direction * ray.t;

            float segmentsX = 5;
            float segmentsY = 5;
            glm::vec3 a = parallelogramLight.v0;
            glm::vec3 b = parallelogramLight.edge01;
            glm::vec3 c = parallelogramLight.edge02;
          
            for (int i = 0; i <= segmentsX; i++) {
                for (int j = 0; j <= segmentsY; j++) {
                    Ray lightRay;
                    lightRay.origin = a + b * float(i / segmentsX) + c * float(j / segmentsY);
                    lightRay.direction = vertex - lightRay.origin;
                    HitInfo info;
                    std::vector<float> arr;
                    if (bvh.intersect(lightRay, info)) {
                        glm::vec3 rayColor = parallelogramLight.color0 * (1.0f - float(i / segmentsX)) * (1.0f - float(j / segmentsY)) +
                            parallelogramLight.color1 * float(i / segmentsX) * (1.0f - float(j / segmentsY)) +
                            parallelogramLight.color2 * (1.0f - float(i / segmentsX)) * float(j / segmentsY) +
                            parallelogramLight.color3 * float(i / segmentsX) * float(j / segmentsY);
                        PointLight pl = { lightRay.origin, rayColor };
                        drawRay(lightRay, rayColor);
                        glm::vec3 specular = 1/(segmentsX * segmentsY) * calculateSpecular(hitInfo, pl, ray);
                        glm::vec3 diffuse = 1/(segmentsX * segmentsY) * calculateDiffuse(hitInfo, pl, ray);
                        if (pointInTheLight(pl, ray, hitInfo, bvh)) {
                            color += specular;
                            color += diffuse;
                        }
                    }
                }
            }
        }
    }
    return glm::clamp(color, 0.0f, 1.0f);
}

bool pointInTheLight(PointLight pointLight, Ray ray, HitInfo hitInfo, BoundingVolumeHierarchy bvh) {
    glm::vec3 vertex = ray.origin + ray.direction * ray.t;
    glm::vec3 light = glm::normalize(pointLight.position - vertex);

    Ray newRay;
    newRay.origin = vertex;
    newRay.direction = glm::normalize(light);
    float initialT = glm::distance(newRay.origin, pointLight.position);
    newRay.t = initialT;

    
    //std::cout << glm::dot(newRay.direction, glm::normalize(glm::vec3(-1) * ray.direction)) << '\n';

    HitInfo lightHitInfo;
    std::vector<float> arr;

    if (bvh.intersect(newRay, lightHitInfo) && newRay.t < initialT) {
        // The ray intersects something
        // The ray should not contribute to the shading of the point
        
        // Visual Debug
        //drawRay(newRay, glm::vec3(1.0f, 0.0f, 1.0f));

        return false;

    }
    // Visual Debug
    //drawRay(newRay, glm::vec3(0.0f, 1.0f, 0.0f));
    return true;
}
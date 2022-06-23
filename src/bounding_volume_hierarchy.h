#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include "framework/mesh.h"
#include <array>
#include <span>

struct Node {
    bool instantiated = false;
    bool isLeaf = true;
    std::vector<int> children;
    std::vector<Mesh> meshes;
    AxisAlignedBox aabb = AxisAlignedBox(glm::vec3(0.0f), glm::vec3(0.0f));
};

struct rayAABBintersect {
    float t;
    int nodeIndex;
    bool operator<(const rayAABBintersect &other) const
    {
        return t < other.t;
    }
};


struct distanceAABB {
    float t;
    int nodeIndex;
};

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Implement these two functions for the Visual Debug.
    // The first function should return how many levels there are in the tree that you have constructed.
    // The second function should draw the bounding boxes of the nodes at the selected level.
    int numLevels() const;
    void debugDraw(int level);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    // bool intersect(Ray& ray, HitInfo& hitInfo, int nodeIndex, std::vector<float>& rayIntersections) const;
    bool intersect(Ray& ray, HitInfo& hitInfo) const;
    //bool intersect(Ray& ray, HitInfo& hitInfo) const;
    glm::vec3 computeCentroid(Vertex v0, Vertex v1, Vertex v2);

private:
    Scene* m_pScene;
    int levels;
    int levelsReached = 0;
    std::vector<Node> tree;
    //std::vector<distanceAABB> rayIntersections;
    void updateXYZ(Vertex v0, Vertex v1, Vertex v2,
        float& minX, float& minY, float& minZ,
        float& maxX, float& maxY, float& maxZ);
    int numTriInNode(int nodeIndex);
    void updateAABB(Vertex v0, Vertex v1, Vertex v2, int nodeIndex);
    void splitNode(int nodeIndex);
    void splitNodeMesh(int nodeIndex, AxisAlignedBox leftBox, AxisAlignedBox rightBox);
    glm::vec3 computeMedianCentroid(int nodeIndex);
    void computeMinMaxPoints(int nodeIndex, glm::vec3& minPoint, glm::vec3& maxPoint);
    bool comp(distanceAABB& a, distanceAABB& b);
    void intersectRayWithAABB(distanceAABB dist);
};

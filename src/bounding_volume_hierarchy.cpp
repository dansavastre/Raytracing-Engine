#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <queue>
#include <math.h>
#include <algorithm>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    levels = 3;
    tree.resize(pow(2, levels));
    //rayIntersections.resize(1);

    float minX = 1000;
    float minY = 1000;
    float minZ = 1000;

    float maxX = -1000;
    float maxY = -1000;
    float maxZ = -1000;
    for (const auto& mesh : m_pScene->meshes) {
        Mesh newMesh;
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];

           updateXYZ(v0, v1, v2, minX, minY, minZ, maxX, maxY, maxZ);

            newMesh.vertices.push_back(v0);
            newMesh.vertices.push_back(v1);
            newMesh.vertices.push_back(v2);
            newMesh.material = mesh.material;
            int triIndex = newMesh.vertices.size() - 3;
            newMesh.triangles.push_back(glm::uvec3(triIndex, triIndex + 1, triIndex + 2));
        }
        tree[0].meshes.push_back(newMesh);
    }
    glm::vec3 minPoint = glm::vec3(minX, minY, minZ);
    glm::vec3 maxPoint = glm::vec3(maxX, maxY, maxZ);
    tree[0].aabb = AxisAlignedBox(minPoint, maxPoint);
    tree[0].instantiated = true;

    if (numTriInNode(0) > 1) {
        tree[0].isLeaf = false;
        splitNode(0);
    }

    levels = levelsReached + 1;
    std::cout << "Levels: " << levels << '\n';
    //std::cout << "Scene meshes: " << pScene->meshes.size() << ", Bvh meshes: " << meshIndex << '\n';
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display.
int BoundingVolumeHierarchy::numLevels() const
{
    return levels;
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    int numTri = 0;
    for (int i = 0; i < tree.size(); i++) {
        int nodeLevel = floor(log2(i + 1));
        if (nodeLevel == level && tree[i].instantiated) {
            numTri += numTriInNode(i);
            drawAABB(tree[i].aabb, DrawMode::Wireframe);
        }
    }
    //std::cout << numTri << '\n';
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .

//bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
//{
//    bool hit = false;
//    // Intersect with all triangles of all meshes.
//    for (const auto& mesh : m_pScene->meshes) {
//        for (const auto& tri : mesh.triangles) {
//            const auto v0 = mesh.vertices[tri[0]];
//            const auto v1 = mesh.vertices[tri[1]];
//            const auto v2 = mesh.vertices[tri[2]];
//            if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
//                //std::cout << v0.position.x << ' ' << v0.position.y << ' ' << v0.position.z << '\n' <<
//                //    v1.position.x << ' ' << v1.position.y << ' ' << v1.position.z << '\n' <<
//                //    v2.position.x << ' ' << v2.position.y << ' ' << v2.position.z << '\n' << '\n';
//                hitInfo.material = mesh.material;
//                hit = true;
//            }
//        }
//    }
//    // Intersect with spheres.
//    for (const auto& sphere : m_pScene->spheres) {
//        hit |= intersectRayWithShape(sphere, ray, hitInfo);
//    }
//        
//    return hit;
//}


bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    float minT = ray.t;
    Ray initialRay = ray;
    bool hit = false;
    std::priority_queue<rayAABBintersect> queue;
    if(intersectRayWithShape(tree[0].aabb, ray)) {
        queue.push({ray.t, 0});
    }
    while(!queue.empty()) {
        rayAABBintersect top = queue.top();
        queue.pop();
        int nodeIndex = top.nodeIndex;

        if(tree[nodeIndex].instantiated) {
            // check if node is a leaf
            ray.t = initialRay.t;
            if(tree[nodeIndex].isLeaf) {
                for (const auto& mesh : tree[nodeIndex].meshes) {
                    for (const auto& tri : mesh.triangles) {
                        const auto v0 = mesh.vertices[tri[0]];
                        const auto v1 = mesh.vertices[tri[1]];
                        const auto v2 = mesh.vertices[tri[2]];

                        if (intersectRayWithTriangle(v0, v1, v2, ray, hitInfo)) {
                            // Update ray.t
                            minT = std::min(minT, ray.t);
                            hitInfo.material = mesh.material;
                            hit = true;
                            
                        }
                    }
                }
            } 
            else {
                int leftIndex = 2 * nodeIndex + 1;
                int rightIndex = 2 * nodeIndex + 2;
                float leftT = initialRay.t;
                float rightT = initialRay.t;
                
                if (tree[leftIndex].instantiated) {
                    if(intersectRayWithShape(tree[leftIndex].aabb, ray)) {
                        queue.push(rayAABBintersect {ray.t, leftIndex});
                    }
                }
                ray.t = initialRay.t;
                if (tree[rightIndex].instantiated) {
                    if(intersectRayWithShape(tree[rightIndex].aabb, ray)) {
                        queue.push(rayAABBintersect {ray.t, rightIndex});
                    }
                }
            }
        }
    }
    ray.t = minT;
    return hit;
}

bool BoundingVolumeHierarchy::comp(distanceAABB& a, distanceAABB& b) {
    if (a.t < b.t) {
        return true;
    }
    else if (a.t == b.t) {
        return a.nodeIndex < b.nodeIndex;
    }
    return false;
}

glm::vec3 BoundingVolumeHierarchy::computeCentroid(Vertex v0, Vertex v1, Vertex v2) {
    return glm::vec3((v0.position.x + v1.position.x + v2.position.x) / 3,
        (v0.position.y + v1.position.y + v2.position.y) / 3,
        (v0.position.z + v1.position.z + v2.position.z) / 3);
}

void BoundingVolumeHierarchy::updateXYZ(Vertex v0, Vertex v1, Vertex v2,
    float& minX, float& minY, float& minZ,
    float& maxX, float& maxY, float& maxZ) {
    // Update x, y, z
    minX = std::min(minX, std::min(v0.position.x, std::min(v1.position.x, v2.position.x)));
    minY = std::min(minY, std::min(v0.position.y, std::min(v1.position.y, v2.position.y)));
    minZ = std::min(minZ, std::min(v0.position.z, std::min(v1.position.z, v2.position.z)));

    maxX = std::max(maxX, std::max(v0.position.x, std::max(v1.position.x, v2.position.x)));
    maxY = std::max(maxY, std::max(v0.position.y, std::max(v1.position.y, v2.position.y)));
    maxZ = std::max(maxZ, std::max(v0.position.z, std::max(v1.position.z, v2.position.z)));
}

int BoundingVolumeHierarchy::numTriInNode(int nodeIndex) {
    int ans = 0;
    for (const auto& mesh : tree[nodeIndex].meshes) {
        ans += mesh.triangles.size();
    }
    return ans;
}

glm::vec3 BoundingVolumeHierarchy::computeMedianCentroid(int nodeIndex) {
    glm::vec3 medianCentroid = glm::vec3(0.0f);
    float numTri = 0;
    for (const auto& mesh : tree[nodeIndex].meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];

            medianCentroid += computeCentroid(v0, v1, v2);
            numTri++;
        }
    }
    return medianCentroid / numTri;
}

void BoundingVolumeHierarchy::computeMinMaxPoints(int nodeIndex, glm::vec3& minPoint, glm::vec3& maxPoint) {
    float minX = 1000;
    float minY = 1000;
    float minZ = 1000;

    float maxX = -1000;
    float maxY = -1000;
    float maxZ = -1000;

    for (const auto& mesh : tree[nodeIndex].meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];

            updateXYZ(v0, v1, v2, minX, minY, minZ, maxX, maxY, maxZ);
        }
    }
    minPoint = glm::vec3(minX, minY, minZ);
    maxPoint = glm::vec3(maxX, maxY, maxZ);
}

void BoundingVolumeHierarchy::updateAABB(Vertex v0, Vertex v1, Vertex v2, int nodeIndex) {
    tree[nodeIndex].aabb.lower.x = std::min(tree[nodeIndex].aabb.lower.x, std::min(v0.position.x, std::min(v1.position.x, v2.position.x)));
    tree[nodeIndex].aabb.lower.y = std::min(tree[nodeIndex].aabb.lower.y, std::min(v0.position.y, std::min(v1.position.y, v2.position.y)));
    tree[nodeIndex].aabb.lower.z = std::min(tree[nodeIndex].aabb.lower.z, std::min(v0.position.z, std::min(v1.position.z, v2.position.z)));

    tree[nodeIndex].aabb.upper.x = std::max(tree[nodeIndex].aabb.upper.x, std::max(v0.position.x, std::max(v1.position.x, v2.position.x)));
    tree[nodeIndex].aabb.upper.y = std::max(tree[nodeIndex].aabb.upper.y, std::max(v0.position.y, std::max(v1.position.y, v2.position.y)));
    tree[nodeIndex].aabb.upper.z = std::max(tree[nodeIndex].aabb.upper.z, std::max(v0.position.z, std::max(v1.position.z, v2.position.z)));
}

void BoundingVolumeHierarchy::splitNodeMesh(int nodeIndex, AxisAlignedBox leftBox, AxisAlignedBox rightBox) {
    int meshIndex = 0;
    int triIndex;
    int leftChild = 2 * nodeIndex + 1;
    int rightChild = 2 * nodeIndex + 2;

    for (const auto& mesh : tree[nodeIndex].meshes) {
        Mesh leftMesh;
        Mesh rightMesh;

        bool addLeft = false;
        bool addRight = false;
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];

            glm::vec3 centroid = computeCentroid(v0, v1, v2);

            // Check if tri should go in left child
            if ((centroid.x >= leftBox.lower.x && centroid.x <= leftBox.upper.x) &&
                (centroid.y >= leftBox.lower.y && centroid.y <= leftBox.upper.y) &&
                (centroid.z >= leftBox.lower.z && centroid.z <= leftBox.upper.z)) {

                addLeft = true;
                leftMesh.vertices.push_back(v0);
                leftMesh.vertices.push_back(v1);
                leftMesh.vertices.push_back(v2);
                leftMesh.material = mesh.material;
                triIndex = leftMesh.vertices.size() - 3;
                leftMesh.triangles.push_back(glm::uvec3(triIndex, triIndex + 1, triIndex + 2));
            }

            // Check if tri should go in right child
            if ((centroid.x >= rightBox.lower.x && centroid.x <= rightBox.upper.x) &&
                (centroid.y >= rightBox.lower.y && centroid.y <= rightBox.upper.y) &&
                (centroid.z >= rightBox.lower.z && centroid.z <= rightBox.upper.z)) {

                addRight = true;
                rightMesh.vertices.push_back(v0);
                rightMesh.vertices.push_back(v1);
                rightMesh.vertices.push_back(v2);
                rightMesh.material = mesh.material;
                triIndex = rightMesh.vertices.size() - 3;
                rightMesh.triangles.push_back(glm::uvec3(triIndex, triIndex + 1, triIndex + 2));
            }
        }
        if (addLeft) {
            tree[leftChild].meshes.push_back(leftMesh);
        }
        if (addRight) {
            tree[rightChild].meshes.push_back(rightMesh);
        }
        if (!addLeft && !addRight) {
            std::cout << "Lost triangle\n";
        }
    }
}

void BoundingVolumeHierarchy::splitNode(int nodeIndex) {

    int level = floor(log2(nodeIndex + 1));
    levelsReached = std::max(level, levelsReached);

    tree[nodeIndex].instantiated = true;
    glm::vec3 medianCentroid = computeMedianCentroid(nodeIndex);

    glm::vec3 minPoint = glm::vec3(1000.0f);
    glm::vec3 maxPoint = glm::vec3(-1000.0f);
    computeMinMaxPoints(nodeIndex, minPoint, maxPoint);
    tree[nodeIndex].aabb = AxisAlignedBox(minPoint, maxPoint);

    glm::vec3 lowerMedian;
    glm::vec3 upperMedian;
    switch (level % 3) {
    case(0):
        // Split by X
        lowerMedian = glm::vec3(medianCentroid.x, tree[nodeIndex].aabb.lower.y, tree[nodeIndex].aabb.lower.z);
        upperMedian = glm::vec3(medianCentroid.x, tree[nodeIndex].aabb.upper.y, tree[nodeIndex].aabb.upper.z);
        break;
    case(1):
        // Split by Y
        lowerMedian = glm::vec3(tree[nodeIndex].aabb.lower.x, medianCentroid.y, tree[nodeIndex].aabb.lower.z);
        upperMedian = glm::vec3(tree[nodeIndex].aabb.upper.x, medianCentroid.y, tree[nodeIndex].aabb.upper.z);
        break;
    case(2):
        // Split by Z
        lowerMedian = glm::vec3(tree[nodeIndex].aabb.lower.x, tree[nodeIndex].aabb.lower.y, medianCentroid.z);
        upperMedian = glm::vec3(tree[nodeIndex].aabb.upper.x, tree[nodeIndex].aabb.upper.y, medianCentroid.z);
        break;
    }
    tree[2 * nodeIndex + 1].aabb = AxisAlignedBox(minPoint, upperMedian);
    tree[2 * nodeIndex + 2].aabb = AxisAlignedBox(lowerMedian, maxPoint);

    splitNodeMesh(nodeIndex, tree[2 * nodeIndex + 1].aabb, tree[2 * nodeIndex + 2].aabb);

    tree[2 * nodeIndex + 1].instantiated = true;
    tree[2 * nodeIndex + 2].instantiated = true;

    // Check to see if children need to be slit up
    if (level + 2 < levels && numTriInNode(2 * nodeIndex + 1) > 1) {
        tree[nodeIndex].isLeaf = false;
        splitNode(2 * nodeIndex + 1);
    }
    if (level + 2 < levels && numTriInNode(2 * nodeIndex + 2) > 1) {
        tree[nodeIndex].isLeaf = false;
        splitNode(2 * nodeIndex + 2);
    }

}

#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <span>

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Implement these two functions for the Visual Debug.
    // The first function should return how many levels there are in the tree that you have constructed.
    // The second function should draw the bounding boxes of the nodes at the selected level.
    int numLevels() const;
    void debugDraw(int level);

    int numLeafNodes() const;
    void drawLeafs(int index);

    void setNumBins(int n);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo,const VisualDebug& visualDebug) const;


    struct Node {
        // boolean that specifies whether the node is a leaf or not
        bool isLeaf;

        // When a node is a leaf, the vector 'indices' will correspond to a single triangle, 
        // such that indices[0][0] contains the index of the belonging mesh in scene.meshes, 
        // and indices[0][1] contains the index of the triangle in that mesh.
        // In case of a non-leaf node, indices will 'point' to the node's children,
        // such that indices[0][0] will contain the index of the left child in vector 'nodes',
        // and indices[1][0] will contain the index of the right child in vector 'nodes'.
        std::vector<std::vector<int>> indices;

        // this is the AxisAlignedBox that is fully filled with all the triangles of the leafs of the node's subtree
        AxisAlignedBox box;

        // the level of the node inside the tree hierarchy, i.e. root is level 0, his children are level 1, etc.
        int level;
    };

    //void loadVertexNormals() const;

    //Basic traversal
    bool intersectCustomBasic(Ray& ray, HitInfo& hitInfo, float depth, int nodeIndex, const VisualDebug& visualDebug, glm::vec3 recursiveColor) const;
    //Advanced traversal
    bool intersectCustomAdvanced(Ray& ray, HitInfo& hitInfo, float depth, int nodeIndex, const VisualDebug& visualDebug) const;
    //Iterative (ironically) part for advanced traversal
    bool intersectCustomRecursive(Ray& ray, HitInfo& hitInfo, std::tuple<int, float>  currentPopped, const VisualDebug& visualDebug, glm::vec3 recursiveColor) const;

private:
    Scene* m_pScene;
};

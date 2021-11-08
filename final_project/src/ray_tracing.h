#pragma once
#include "scene.h"

struct HitInfo {
    glm::vec3 planeNormal; 
    glm::vec3 interpolatedNormal;
    Material material;
    Vertex v0, v1, v2;
    glm::vec3 intersectionPoint;
};

//Structure Bolek made to save any checkboxes in one place and reference them in any method
//This allows the user to select what features / visual debugs they want to make use of
struct VisualDebug {
    bool shadingAndIntersection;
    bool recursiveRayTracer;
    bool TraversalAdvancedSelected;
    bool TraversalBasic;
    bool TraversalAdvanced;
    bool Transparency;
    bool transparency;
    bool hardShadows;
    bool segmentLightSourceShadows;
    bool parallelogramLightSourceShadows;
    bool interpolatedNormal;
    bool glossyReflections;
    bool multipleRaysPerPixel;
    bool bloomOn;
    bool notVisitedNodesTraversal;
    bool bloomDebug;
    int bloomStage;
    bool drawIntersectedButNotVisited;
};

bool intersectRayWithPlane(const Plane& plane, Ray& ray);

// Returns true if the point p is inside the triangle spanned by v0, v1, v2 with normal n.
bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p);

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2);

//changed to Vertex from glm::vec3
bool intersectRayWithTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, Ray& ray, HitInfo& hitInfo);
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo);
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray);

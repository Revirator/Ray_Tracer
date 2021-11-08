#pragma once
#include <framework/disable_all_warnings.h>
#include "bounding_volume_hierarchy.h""
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()

bool hardShadows(const BoundingVolumeHierarchy& bvh, const Ray& ray, const PointLight& pointLight, const VisualDebug& visualDebug);

glm::vec3 segmentLightColour(const BoundingVolumeHierarchy& bvh, Ray ray, const SegmentLight& segmentLight, HitInfo& hitInfo, const VisualDebug& visualDebug);

glm::vec3 parallelogramLightColour(const BoundingVolumeHierarchy& bvh, Ray ray, const ParallelogramLight& parallelogramLight, HitInfo& hitInfo, const VisualDebug& visualDebug);

//Imported from assignment 5. Calculates specularity
glm::vec3 phongSpecularOnly(const Material& shadingData, const glm::vec3& vertexPos, const glm::vec3& normal, const glm::vec3& lightPos, const glm::vec3& cameraPos, const glm::vec3& lightColor);

//Imported from assignment 5. Calculates diffuse
glm::vec3 diffuseOnly(const Vertex& v0, const Vertex& v1, const Vertex& v2, const Material& shadingData, const glm::vec3& vertexPos, const glm::vec3& normal, const glm::vec3& lightPos, const glm::vec3& lightColor);


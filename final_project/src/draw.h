#pragma once
#include "scene.h"
#include <framework/mesh.h>
#include <framework/ray.h>
#include <utility> // std::forward

enum class DrawMode {
    Filled,
    Wireframe
};


// Add your own custom visual debug draw functions here then implement it in draw.cpp.
// You are free to modify the example one however you like.
void drawExampleOfCustomVisualDebug();

extern bool enableDrawRay;
void drawRay(const Ray& ray, const glm::vec3& color = glm::vec3(1.0f));

void drawAABB(const AxisAlignedBox& box, DrawMode drawMode = DrawMode::Filled, const glm::vec3& color = glm::vec3(1.0f), float transparency = 1.0f);

void drawMesh(const Mesh& mesh);
//this method allows to draw a triangle of the color red (used by Bolek)
void drawTriangle(const Mesh& mesh, int index);
void drawTriangleColor(const Mesh& mesh, int index, glm::vec3 color);
void drawSphere(const Sphere& sphere);
void drawSphere(const glm::vec3& center, float radius, const glm::vec3& color = glm::vec3(1.0f));
void drawScene(const Scene& scene);


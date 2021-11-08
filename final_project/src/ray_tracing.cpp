#include "ray_tracing.h"
#include "draw.h"
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

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    //Calculate intersection Plane-Ray
    double UpperPart = (double)plane.D - glm::dot(ray.origin, plane.normal);
    double BottomPart = (double)glm::dot(ray.direction, plane.normal);
    if (BottomPart == 0) {
        return false;
    }
    double t = UpperPart / BottomPart;
    //Condition of tutorial. This also makes sure the ray hits the closest object only
    if (t <= 0 || ray.t < t) {
        return false;
    }
    ray.t = t;
    return true;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    plane.normal = glm::normalize(glm::cross(v0 - v2, v1 - v2));
    plane.D = glm::dot(v0, plane.normal);
    return plane;
}

// Changed from glm::vec3 to vertex
bool intersectRayWithTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, Ray& ray, HitInfo& hitInfo)
{
    Plane plane = trianglePlane(v0.position, v1.position, v2.position);
    float old_t = ray.t;
    bool isInPlane = intersectRayWithPlane(plane, ray);
    glm::vec3 p = ray.origin + ray.t * ray.direction;
    if (isInPlane) {
        float areaABC = glm::dot(plane.normal, glm::cross((v1.position - v0.position), (v2.position - v0.position)));
        float areaPBC = glm::dot(plane.normal, glm::cross((v1.position - p), (v2.position - p)));
        float areaPCA = glm::dot(plane.normal, glm::cross((v2.position - p), (v0.position - p)));

        float alpha = areaPBC / areaABC;
        float beta = areaPCA / areaABC;
        float gamma = 1 - alpha - beta;

        bool inTriangle = alpha >= -0.00001 && beta >= -0.00001 && alpha + beta <= 1.00001;
        if (inTriangle) {
            // Calculate interpolated normal instead of the plane normal
            hitInfo.planeNormal = plane.normal;
            hitInfo.interpolatedNormal = glm::normalize(alpha * v0.normal + beta * v1.normal + gamma * v2.normal);
            hitInfo.v0 = v0;
            hitInfo.v1 = v1;
            hitInfo.v2 = v2;
            hitInfo.intersectionPoint = p;
            return true;
        }
    }
    //Reset the ray's t in case we don't have an intersection with the triangle
    ray.t = old_t;
    return false;
}

bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    float A = powf(ray.direction.x, 2) + powf(ray.direction.y, 2) + powf(ray.direction.z, 2);
    float B = 2 * (ray.direction.x * (ray.origin.x - sphere.center.x) + ray.direction.y * (ray.origin.y - sphere.center.y) + ray.direction.z * (ray.origin.z - sphere.center.z));
    float C = powf(ray.origin.x - sphere.center.x, 2) + powf(ray.origin.y - sphere.center.y, 2) + powf(ray.origin.z - sphere.center.z, 2) - powf(sphere.radius, 2);
    float D = powf(B, 2) - 4 * A * C;
    if (D < 0)
    {
        return false;
    }
    if (D == 0)
    {
        float new_t = (-B + sqrt(D)) / (2 * A);
        if (new_t < ray.t && new_t > 0)
        {
            ray.t = new_t;
        }
        glm::vec3 rayPoint = ray.origin + new_t * ray.direction;
        glm::vec3 hitNormal = glm::normalize(rayPoint - sphere.center);
        hitInfo.planeNormal = hitNormal;
        hitInfo.interpolatedNormal = hitNormal;
        hitInfo.material = sphere.material;
        return true;
    }
    float new_t_1 = (-B - sqrt(D)) / (2 * A);
    float new_t_2 = (-B + sqrt(D)) / (2 * A);
    if (new_t_1 < new_t_2)
    {
        if (new_t_1 > 0)
        {
            if (new_t_1 < ray.t)
            {
                ray.t = new_t_1;
                glm::vec3 rayPoint = ray.origin + new_t_1 * ray.direction;
                glm::vec3 hitNormal = glm::normalize(rayPoint - sphere.center);
                hitInfo.planeNormal = hitNormal;
                hitInfo.interpolatedNormal = hitNormal;
                hitInfo.material = sphere.material;
                return true;
            }
        }
        else if (new_t_2 > 0 && new_t_2 < ray.t)
        {
            ray.t = new_t_2;
            glm::vec3 rayPoint = ray.origin + new_t_2 * ray.direction;
            glm::vec3 hitNormal = glm::normalize(rayPoint - sphere.center);
            hitInfo.planeNormal = hitNormal;
            hitInfo.interpolatedNormal = hitNormal;
            hitInfo.material = sphere.material;
            return true;
        }
    }
    else
    {
        if (new_t_2 > 0)
        {
            if (new_t_2 < ray.t)
            {
                ray.t = new_t_2;
                glm::vec3 rayPoint = ray.origin + new_t_2 * ray.direction;
                glm::vec3 hitNormal = glm::normalize(rayPoint - sphere.center);
                hitInfo.planeNormal = hitNormal;
                hitInfo.interpolatedNormal = hitNormal;
                hitInfo.material = sphere.material;
                return true;
            }
        }
        else if (new_t_1 > 0 && new_t_1 < ray.t)
        {
            ray.t = new_t_1;
            glm::vec3 rayPoint = ray.origin + new_t_1 * ray.direction;
            glm::vec3 hitNormal = glm::normalize(rayPoint - sphere.center);
            hitInfo.planeNormal = hitNormal;
            hitInfo.interpolatedNormal = hitNormal;
            hitInfo.material = sphere.material;
            return true;
        }
    }
    return false;
}

//This is a custom interpretation of a formula on the slides of Week 5 (lec. 1)
//It uses properties of the AAB to easily calculate a correct t (depending on axis)
double calculateAABBt(double planePoint, Ray& ray, char axis) {
    switch (axis) {
    case 'x':
        return (planePoint - (double)ray.origin.x) / (double)ray.direction.x;
    case 'y':
        return (planePoint - (double)ray.origin.y) / (double)ray.direction.y;
    case 'z':
        return (planePoint - (double)ray.origin.z) / (double)ray.direction.z;
    }
    return -69.0;
}

bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    //Save oldest t and old t (same value) in case of rollbacks
    float oldest_t = ray.t;
    float old_t = ray.t;
    //check where ray intersects every axis
    double tx1 = calculateAABBt((double)box.lower.x, ray, 'x');
    //Revert to old_t just to make sure you don't screw up
    ray.t = old_t;
    double tx2 = calculateAABBt((double)box.upper.x, ray, 'x');
    ray.t = old_t;
    //Minimum value is the entry point, maximum value the "leaving" point of the cube
    double tx_in = (double)glm::min(tx1, tx2);
    double tx_out = (double)glm::max(tx1, tx2);

    double ty1 = calculateAABBt((double)box.lower.y, ray, 'y');
    ray.t = old_t;
    double ty2 = calculateAABBt((double)box.upper.y, ray, 'y');
    ray.t = old_t;
    double ty_in = (double)glm::min(ty1, ty2);
    double ty_out = (double)glm::max(ty1, ty2);

    double tz1 = calculateAABBt((double)box.lower.z, ray, 'z');
    ray.t = old_t;
    double tz2 = calculateAABBt((double)box.upper.z, ray, 'z');
    ray.t = old_t;
    double tz_in = (double)glm::min(tz1, tz2);
    double tz_out = (double)glm::max(tz1, tz2);

    //Calculate global points of entry and exit
    double t_global_in = (double)glm::max((double)glm::max(tx_in, ty_in), tz_in);
    double t_global_out = (double)glm::min((double)glm::min(tx_out, ty_out), tz_out);

    //If exit before entry, return false
    if (t_global_in > t_global_out || t_global_out < 0 || ray.t < t_global_in) {
        ray.t = oldest_t;
        return false;
    }

    //Special case. If we are inside of the box we use global_out as our t
    if (box.lower.x < ray.origin.x && ray.origin.x < box.upper.x && box.lower.y < ray.origin.y && ray.origin.y < box.upper.y && box.lower.z < ray.origin.z && ray.origin.z < box.upper.z) {
        ray.t = t_global_out;
        return true;
    }

    //adjust ray to point of global entry and return true
    ray.t = (float)t_global_in;
    return true;
    //return false;
}


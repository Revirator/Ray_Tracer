#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "ray_tracing.h"
#include "shadows.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
DISABLE_WARNINGS_POP()
#include <cstdlib>
#include <random>
#include <string>
#include <iostream>
#include <type_traits>
#include <chrono>
#include <variant>

// Given a point light source and a ray compute the intersection point and return true if it is in the shadow and false otherwise.
bool hardShadows(const BoundingVolumeHierarchy& bvh, const Ray& ray, const PointLight& pointLight, const VisualDebug& visualDebug) {
    HitInfo filler;
    bool inShadow = false;
    glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
    glm::vec3 directionTowardsTheLight = glm::normalize(pointLight.position - intersectionPoint);
    // Multiplying with 0.0001 (epsilon) because of numerical imprecision that can result in an intersection
    // with the surface on which the point of intersection lies
    // Source: p.86 from the book
    Ray rayTowardsLight(intersectionPoint + (0.0001f * directionTowardsTheLight), directionTowardsTheLight, glm::length(pointLight.position - intersectionPoint));
    if (bvh.intersect(rayTowardsLight, filler, visualDebug)) {
        // When true => there is something in the the way => in shadow
        inShadow = true;
        if(visualDebug.hardShadows) {
            // Draw a red ray towards the light (for the visual debugger)
            drawRay(rayTowardsLight, glm::vec3(1.0f, 0.0f, 0.0f));
        }
    } else {
        // When false => the ray reaches the light => not in shadow
        inShadow = false;
        if(visualDebug.hardShadows) {
            // Draw a green ray towards the light (for the visual debugger)
            drawRay(rayTowardsLight, glm::vec3(0.0f, 1.0f, 0.0f));
        }
    }
    return inShadow;
}

glm::vec3 segmentLightColour(const BoundingVolumeHierarchy& bvh, Ray ray, const SegmentLight& segmentLight, HitInfo& hitInfo, const VisualDebug& visualDebug) {
    HitInfo filler;
    glm::vec3 colour(0.0f);
    glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
    glm::vec3 directionTowardsEndpoint0 = glm::normalize(segmentLight.endpoint0 - intersectionPoint);
    glm::vec3 directionTowardsEndpoint1 = glm::normalize(segmentLight.endpoint1 - intersectionPoint);
    Ray rayTowardsEndpoint0(intersectionPoint + (0.0001f * directionTowardsEndpoint0), directionTowardsEndpoint0, glm::length(segmentLight.endpoint0 - intersectionPoint));
    Ray rayTowardsEndpoint1(intersectionPoint + (0.0001f * directionTowardsEndpoint1), directionTowardsEndpoint1, glm::length(segmentLight.endpoint1 - intersectionPoint));
    // segmentLine is the coloured line between the 2 endpoints
    // origin of the vector is segmentLight.endpoint0
    glm::vec3 segmentLine = segmentLight.endpoint1 - segmentLight.endpoint0;

    // REGULAR SAMPLING ----------------------------------------------------------------

    // Change the increment of i for more samples
//    for (float i = 0.0f; i <= 1.00001f; i += 0.005f) {
//        glm::vec3 pointOnSegment = (i * segmentLine) + segmentLight.endpoint0;
//        glm::vec3 directionTowardsPointOnSegment = glm::normalize(pointOnSegment - intersectionPoint);
//        Ray rayTowardsPointOnSegment(intersectionPoint + (0.0001f * directionTowardsPointOnSegment), directionTowardsPointOnSegment, glm::length(pointOnSegment - intersectionPoint));
//        if (bvh.intersect(rayTowardsPointOnSegment, filler)) {
//            // Visual Debugger 3.4
//            if(visualDebug.segmentLightSourceShadows) {
//                // draw a white ray towards the light (for the visual debugger)
//                drawRay(rayTowardsPointOnSegment, glm::vec3(1.0f, 1.0f, 1.0f));
//            }
//        }
//        else {
//            // Calculate colour for the current point on the segment
//            glm::vec3 segmentColour = i * segmentLight.color1 + (1 - i) * segmentLight.color0;
//            glm::vec3 localNormal = hitInfo.interpolatedNormal;
//            if (glm::dot(localNormal, glm::normalize(pointOnSegment - intersectionPoint)) < 0) {
//                localNormal = -hitInfo.interpolatedNormal;
//            }
//            if (glm::dot(localNormal, -ray.direction) > 0) {
//                colour += phongSpecularOnly(hitInfo.material, intersectionPoint, localNormal, pointOnSegment, ray.origin, segmentColour);
//                colour += diffuseOnly(hitInfo.material, intersectionPoint, localNormal, pointOnSegment, segmentColour);
//            }
//            if (visualDebug.transparency && hitInfo.material.transparency != 1.0f && glm::dot(hitInfo.planeNormal, -ray.direction) < 0) {
//                colour += phongSpecularOnly(hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal, pointOnSegment, ray.origin, segmentColour);
//                colour += diffuseOnly(hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal, pointOnSegment, segmentColour);
//            }
//            // Visual Debugger 3.4
//            if(visualDebug.segmentLightSourceShadows) {
//                // draw a ray with the segmentColour towards the light (for the visual debugger)
//                drawRay(rayTowardsPointOnSegment, segmentColour);
//            }
//        }
//    }
    // Divide by the number of samples the number of samples
    //return colour / 201.0f;
   
    //----------------------------------------------------------------------------------

    // RANDOM SAMPLING------------------------------------------------------------------

    srand(duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    for (int j = 0; j < 200; j++) {
        float i = ((float)rand() / RAND_MAX);
        glm::vec3 pointOnSegment = (i * segmentLine) + segmentLight.endpoint0;
        glm::vec3 directionTowardsPointOnSegment = glm::normalize(pointOnSegment - intersectionPoint);
        Ray rayTowardsPointOnSegment(intersectionPoint + (0.0001f * directionTowardsPointOnSegment), directionTowardsPointOnSegment, glm::length(pointOnSegment - intersectionPoint));
        if (bvh.intersect(rayTowardsPointOnSegment, filler, visualDebug)) {
            // Visual Debugger 3.4
            if(visualDebug.segmentLightSourceShadows) {
                // Draw a white ray towards the light (for the visual debugger)
                drawRay(rayTowardsPointOnSegment, glm::vec3(1.0f, 1.0f, 1.0f));
            }
        }
        else {
            // Calculate colour for the current point on the segment
            glm::vec3 segmentColour = i * segmentLight.color1 + (1 - i) * segmentLight.color0;
            glm::vec3 localNormal = hitInfo.interpolatedNormal;
            if (glm::dot(localNormal, glm::normalize(pointOnSegment - intersectionPoint)) < 0) {
                localNormal = -hitInfo.interpolatedNormal;
            }
            if (glm::dot(localNormal, -ray.direction) > 0) {
                colour += phongSpecularOnly(hitInfo.material, intersectionPoint, localNormal, pointOnSegment, ray.origin, segmentColour);
                colour += diffuseOnly(hitInfo.v0, hitInfo.v1, hitInfo.v2, hitInfo.material, intersectionPoint, localNormal, pointOnSegment, segmentColour);
            }
            if (visualDebug.transparency && hitInfo.material.transparency != 1.0f && glm::dot(hitInfo.planeNormal, -ray.direction) < 0) {
                colour += phongSpecularOnly(hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal, pointOnSegment, ray.origin, segmentColour);
                colour += diffuseOnly(hitInfo.v0, hitInfo.v1, hitInfo.v2, hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal, pointOnSegment, segmentColour);
            }
            // Visual Debugger 3.4
            if(visualDebug.segmentLightSourceShadows) {
                // Draw a ray with the segmentColour towards the light (for the visual debugger)
                drawRay(rayTowardsPointOnSegment, segmentColour);
            }
        }
    }
    // Divide by the number of samples
    return colour / 200.0f;
    //----------------------------------------------------------------------------------
}

glm::vec3 parallelogramLightColour(const BoundingVolumeHierarchy& bvh, Ray ray, const ParallelogramLight& parallelogramLight, HitInfo& hitInfo, const VisualDebug& visualDebug) {
    HitInfo filler;
    glm::vec3 colour(0.0f);
    glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
    glm::vec3 directionTowardsCenter = glm::normalize(parallelogramLight.v0 - intersectionPoint);
    glm::vec3 edge1 = parallelogramLight.edge01;
    glm::vec3 edge2 = parallelogramLight.edge02;

    // REGULAR SAMPLING ----------------------------------------------------------------

    // Change the increment of i and j for more samples
//    for (float i = 0.0f; i <= 1.00001f; i += 0.05f) {
//        for (float j = 0.0f; j <= 1.00001f; j += 0.05f) {
//            glm::vec3 pointOnSegment = (i * edge1)  + (j * edge2) + parallelogramLight.v0;
//            glm::vec3 directionTowardsPointOnSegment = glm::normalize(pointOnSegment - intersectionPoint);
//            Ray rayTowardsPointOnSegment(intersectionPoint + (0.0001f * directionTowardsPointOnSegment), directionTowardsPointOnSegment, glm::length(pointOnSegment - intersectionPoint));
//            if (bvh.intersect(rayTowardsPointOnSegment, filler)) {
//                // Visual Debugger 3.4
//                if(visualDebug.parallelogramLightSourceShadows) {
//                    // draw a white ray towards the light (for the visual debugger)
//                    drawRay(rayTowardsPointOnSegment, glm::vec3(1.0f, 1.0f, 1.0f));
//                }
//            }
//            else {
//                // Calculate colour for the current point on the segment
//                glm::vec3 parallelogramColour = (1 - j) * ((1 - i) * parallelogramLight.color0 + i * parallelogramLight.color1)
//                                                + j * ((1 - i) * parallelogramLight.color2 + i * parallelogramLight.color3);
//                glm::vec3 localNormal = hitInfo.interpolatedNormal;
//                if (glm::dot(localNormal, glm::normalize(pointOnSegment - intersectionPoint)) < 0) {
//                    localNormal = -hitInfo.interpolatedNormal;
//                }
//                if (glm::dot(localNormal, -ray.direction) > 0) {
//                    colour += phongSpecularOnly(hitInfo.material, intersectionPoint, localNormal, pointOnSegment, ray.origin, parallelogramColour);
//                    colour += diffuseOnly(hitInfo.material, intersectionPoint, localNormal, pointOnSegment, parallelogramColour);
//                }
//                if (visualDebug.transparency && hitInfo.material.transparency != 1.0f && glm::dot(hitInfo.planeNormal, -ray.direction) < 0) {
//                    colour += phongSpecularOnly(hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal, pointOnSegment, ray.origin, parallelogramColour);
//                    colour += diffuseOnly(hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal, pointOnSegment, parallelogramColour);
//                }
//                // Visual Debugger 3.4
//                if(visualDebug.parallelogramLightSourceShadows) {
//                    // draw a ray with the segmentColour towards the light (for the visual debugger)
//                    drawRay(rayTowardsPointOnSegment, parallelogramColour);
//                }
//            }
//        }
//    }
    // Divide by the number of samples
    //return colour / 441.0f;

    //----------------------------------------------------------------------------------

    // RANDOM SAMPLING -----------------------------------------------------------------
    std::vector<float>samplesForI;
    std::vector<float>samplesForJ;
    srand(duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    for (int i = 0; i < 300; i++)
    {
        samplesForI.push_back(((float)rand() / RAND_MAX));
        samplesForJ.push_back(((float)rand() / RAND_MAX));
    }
    std:shuffle(std::begin(samplesForI), std::end(samplesForI), std::default_random_engine{});
    for (size_t k = 0; k < 300; k++) {
        float i = samplesForI.at(k);
        float j = samplesForJ.at(k);
        glm::vec3 pointOnSegment = (i * edge1) + (j * edge2) + parallelogramLight.v0;
        glm::vec3 directionTowardsPointOnSegment = glm::normalize(pointOnSegment - intersectionPoint);
        Ray rayTowardsPointOnSegment(intersectionPoint + (0.0001f * directionTowardsPointOnSegment), directionTowardsPointOnSegment, glm::length(pointOnSegment - intersectionPoint));
        if (bvh.intersect(rayTowardsPointOnSegment, filler, visualDebug)) {
            // Visual Debugger 3.4
            if(visualDebug.parallelogramLightSourceShadows) {
                // Draw a white ray towards the light (for the visual debugger)
                drawRay(rayTowardsPointOnSegment, glm::vec3(1.0f, 1.0f, 1.0f));
            }
        }
        else {
            // Calculate the colour for the current point on the segment using bilinear interpolation
            glm::vec3 parallelogramColour = (1 - j) * ((1 - i) * parallelogramLight.color0 + i * parallelogramLight.color1)
                + j * ((1 - i) * parallelogramLight.color2 + i * parallelogramLight.color3);
            glm::vec3 localNormal = hitInfo.interpolatedNormal;
            if (glm::dot(localNormal, glm::normalize(pointOnSegment - intersectionPoint)) < 0) {
                localNormal = -hitInfo.interpolatedNormal;
            }
            if (glm::dot(localNormal, -ray.direction) > 0) {
                colour += phongSpecularOnly(hitInfo.material, intersectionPoint, localNormal, pointOnSegment, ray.origin, parallelogramColour);
                colour += diffuseOnly(hitInfo.v0, hitInfo.v1, hitInfo.v2, hitInfo.material, intersectionPoint, localNormal, pointOnSegment, parallelogramColour);
            }
            if (visualDebug.transparency && hitInfo.material.transparency != 1.0f && glm::dot(hitInfo.planeNormal, -ray.direction) < 0) {
                colour += phongSpecularOnly(hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal, pointOnSegment, ray.origin, parallelogramColour);
                colour += diffuseOnly(hitInfo.v0, hitInfo.v1, hitInfo.v2, hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal, pointOnSegment, parallelogramColour);
            }
            // Visual Debugger 3.4
            if(visualDebug.parallelogramLightSourceShadows) {
                // Draw a ray with the segmentColour towards the light (for the visual debugger)
                drawRay(rayTowardsPointOnSegment, parallelogramColour);
            }
        }
    }
    // Divide by the number of samples
    return colour / 300.0f;
    //----------------------------------------------------------------------------------
}

glm::vec3 phongSpecularOnly(const Material& shadingData, const glm::vec3& vertexPos, const glm::vec3& normal, const glm::vec3& lightPos, const glm::vec3& cameraPos, const glm::vec3& lightColor)
{
    // Calculating view vector
    glm::vec3 V = glm::normalize(cameraPos - vertexPos);
    // Calculating light vector
    glm::vec3 light = glm::normalize(lightPos - vertexPos);
    // Calculating the reflection vector
    glm::vec3 R = reflect(light, normal);
    // Continuing with the Formula provided in the description
    float dotProd = glm::dot(V, R);
    if (dotProd < 0) {
        dotProd = 0;
    }
    return lightColor * shadingData.ks * powf(dotProd, shadingData.shininess);
}

glm::vec3 diffuseOnly(const Vertex& v0, const Vertex& v1, const Vertex& v2, const Material& shadingData, const glm::vec3& vertexPos, const glm::vec3& normal, const glm::vec3& lightPos, const glm::vec3& lightColor)
{
    // Normalize the light vector
    glm::vec3 light = glm::normalize(lightPos - vertexPos);
    // Calculate the dot Product between the light direction and the normal
    float dotProd = glm::dot(normal, light);
    // If that is negative, clamp it to zero
    if (dotProd < 0) {
        dotProd = 0;
    }
    // if material has texture, use texture pixel instead of kd value
    if (shadingData.kdTexture) {

        // compute barycentric coordinates
        float areaABC = glm::dot(normal, glm::cross((v1.position - v0.position), (v2.position - v0.position)));
        float areaPBC = glm::dot(normal, glm::cross((v1.position - vertexPos), (v2.position - vertexPos)));
        float areaPCA = glm::dot(normal, glm::cross((v2.position - vertexPos), (v0.position - vertexPos)));
        float alpha = areaPBC / areaABC;
        float beta = areaPCA / areaABC;
        float gamma = 1.0f - alpha - beta;

        // use barycentric coordinates to get the texture coordinate of vertexPos
        glm::vec3 kd = shadingData.kdTexture->getTexel(v0.texCoord * alpha + v1.texCoord * beta + v2.texCoord * gamma);
        return lightColor * kd * dotProd;
    }

    // Multiply with coefficient
    return lightColor * shadingData.kd * dotProd;
}

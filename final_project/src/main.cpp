#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "ray_tracing.h"
#include "screen.h"
#include "shadows.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>
#include <nfd.h>
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
DISABLE_WARNINGS_POP()
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <framework/image.h>
#include <framework/imguizmo.h>
#include <framework/trackball.h>
#include <framework/variant_helper.h>
#include <framework/window.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#include <variant>
#include <set>

constexpr glm::ivec2 windowResolution { 800, 800 };
const std::filesystem::path dataPath { DATA_DIR };
int depth = 0;

enum class ViewMode {
    Rasterization = 0,
    RayTracing = 1
};

// We have to declare transparency here, as it relies on getFinalColor
glm::vec3 transparency(const Material& shadingData, const glm::vec3& intersectionPoint, const glm::vec3 color, Ray ray, HitInfo& hitInfo, const Scene& scene, const BoundingVolumeHierarchy& bvh, const VisualDebug& visualDebug);
// We have to declare bloom here, as it relies on getFinalColor
void bloom(const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen, const VisualDebug& visualDebug, int filterSize);

static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray,const VisualDebug& visualDebug)
{
    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo, visualDebug)) {
        // Final colour of the pixel. Black by default.
        glm::vec3 colour(0.0f, 0.0f, 0.0f);
        // Go over all the lights (distinguish between them)
        for (const auto& light : scene.lights) {
            //For point lights
            if (std::holds_alternative<PointLight>(light)) {
                // Get light
                const PointLight pointLight = std::get<PointLight>(light);
                // Go into hardShadows method to calculate hard shadows
                // If true don't calculate shading for this light source
                bool inShadow = hardShadows(bvh, ray, pointLight, visualDebug);
                // If something is in a shadow, don't do Phong. Continue with loop
                if (inShadow) {
                    continue;
                }
                // Calculate the intersection point (of ray and object) and the ray towards the light source
                glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
                Ray rayTowardsLight(intersectionPoint, glm::normalize(pointLight.position - intersectionPoint));
                // Add both diffuse and specularity to the vector unless the plane normal is facing away of the camera
                // Also be sure to invert the normal, in case it faces the wrong way (otherwise the object is wrongly interpreted as not lit)
                glm::vec3 localNormal = hitInfo.interpolatedNormal;
                if (glm::dot(localNormal, glm::normalize(pointLight.position - intersectionPoint)) < 0) {
                    localNormal = -hitInfo.interpolatedNormal;
                }
                //Calculate the colour of opaque objects
                if (glm::dot(localNormal, -ray.direction) > 0) {
                    colour += phongSpecularOnly(hitInfo.material, intersectionPoint, localNormal, pointLight.position,
                                                ray.origin, pointLight.color);
                    colour += diffuseOnly(hitInfo.v0, hitInfo.v1, hitInfo.v2, hitInfo.material, intersectionPoint,
                                          localNormal, pointLight.position, pointLight.color);
                }
                //calculate the colour of (semi)transparent objects
                if (visualDebug.transparency && hitInfo.material.transparency != 1.0f && glm::dot(hitInfo.planeNormal, -ray.direction) < 0) {
                    colour += phongSpecularOnly(hitInfo.material, intersectionPoint, hitInfo.interpolatedNormal,pointLight.position, ray.origin, pointLight.color);
                    colour += diffuseOnly(hitInfo.v0, hitInfo.v1, hitInfo.v2, hitInfo.material, intersectionPoint,hitInfo.interpolatedNormal, pointLight.position, pointLight.color);
                }
                if (visualDebug.transparency && hitInfo.material.transparency != 1.0f) {
                    //Calculate transparency for (semi)transparent objects
                    colour = transparency(hitInfo.material, intersectionPoint, colour, ray, hitInfo, scene, bvh,
                                          visualDebug);
                }
            //For segment lights
            } else if (std::holds_alternative<SegmentLight>(light)) {
                //Get light
                const SegmentLight segmentLight = std::get<SegmentLight>(light);
                //Get colour from the segmentLightColourMethod
                colour += segmentLightColour(bvh, ray, segmentLight, hitInfo, visualDebug);
                if (visualDebug.transparency && hitInfo.material.transparency != 1.0f) {
                    //Calculate transparency for (semi)transparent objects
                    colour = transparency(hitInfo.material, hitInfo.intersectionPoint, colour, ray, hitInfo, scene, bvh,visualDebug);
                }
            //For Parallelogram lights
            } else if (std::holds_alternative<ParallelogramLight>(light)) {
                //Get light
                const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
                //Get colour from the segmentLightColourMethod
                colour += parallelogramLightColour(bvh, ray, parallelogramLight, hitInfo, visualDebug);
                if (visualDebug.transparency && hitInfo.material.transparency != 1.0f) {
                    //Calculate transparency for (semi)transparent objects
                    colour = transparency(hitInfo.material, hitInfo.intersectionPoint, colour, ray, hitInfo, scene, bvh, visualDebug);
                }
            }
        }
        // Visual Debugger 3.1; Also works for "multiple rays per pixel"; Also works for "glossy reflections" ??? (not a bug but a feature :))
        if (visualDebug.shadingAndIntersection && !visualDebug.recursiveRayTracer) {
            if (depth == 0) {
                drawRay(ray, colour);
            }
        }
        // Use recursion (glossy reflections)
        if (hitInfo.material.ks.x > 0.00001f && hitInfo.material.ks.y > 0.00001f && hitInfo.material.ks.z > 0.00001f && depth != 8) {
            //Increase depth, calculate intersection point and get reflected ray from object
            depth += 1;
            glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
            glm::vec3 r = glm::reflect(ray.direction, hitInfo.planeNormal);
            // If glossyReflections is enabled use this for the recursion
            if (visualDebug.glossyReflections) {
                // Perturbing the ray as shown in the book (Section 13.4.4)
                // But instead of one perturbed ray take a random sample
                glm::vec3 w = glm::normalize(r);
                glm::vec3 t;
                if (w.x == fmin(fmin(w.x, w.y), w.z)) {
                    t = glm::vec3(1.0f, w.y, w.z);
                } else if (w.y == fmin(fmin(w.x, w.y), w.z)) {
                    t = glm::vec3(w.x, 1.0f, w.z);
                } else if (w.z == fmin(fmin(w.x, w.y), w.z)) {
                    t = glm::vec3(w.x, w.y, 1.0f);
                }
                glm::vec3 u = glm::normalize(glm::cross(t, w));
                glm::vec3 v = glm::cross(u, w);
                srand(duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                for (size_t k = 0; k < 80; k++) {
                    // a = shininess
                    //float i = ((float)rand() / RAND_MAX) * hitInfo.material.shininess - hitInfo.material.shininess / 2.0f;
                    //float j = ((float)rand() / RAND_MAX) * hitInfo.material.shininess - hitInfo.material.shininess / 2.0f;
                    // a = 1/shininess
                    float i = ((float) rand() / RAND_MAX) / hitInfo.material.shininess - 0.5f / (hitInfo.material.shininess);
                    float j = ((float) rand() / RAND_MAX) / hitInfo.material.shininess - 0.5f / (hitInfo.material.shininess);
                    // a = 0.4 / shininess
                    //float i = ((float)rand() / RAND_MAX) / (2.5f * hitInfo.material.shininess) - 0.2f / (hitInfo.material.shininess);
                    //float j = ((float)rand() / RAND_MAX) / (2.5f * hitInfo.material.shininess) - 0.2f / (hitInfo.material.shininess);
                    glm::vec3 r_prime = glm::normalize(r + i * u + j * v);
                    Ray reflectedRay = Ray(intersectionPoint + (0.0001f * r_prime), r_prime);
                    // If the perturbed ray is below the surface of the intersection point return black colour
                    if (glm::dot(r_prime, hitInfo.planeNormal) < 0) {
                        continue;
                    }
                    colour += getFinalColor(scene, bvh, reflectedRay, visualDebug);
                }
                // Divide by the sample size
                colour /= 80.0f;
            }
            else {
                // If glossyReflections is disabled use normal recursion
                // Get reflected ray from object. Add slight epsilon to account for floating point errors
                Ray reflectedRay = Ray(intersectionPoint + (0.0001f * glm::normalize(r)), glm::normalize(r));
                // Call getFinalColor recursively and add the new color to the current one
                colour += getFinalColor(scene, bvh, reflectedRay, visualDebug);
            }
        }
        else {
            // If depth reached or material fully black set depth to 0 and ignore the recursion
            depth = 0;
        }
        // Visual Debugger 3.2 (Only in use when Visual Debugger 3.1 is on)
        // "extend upon the visual debug from the Ray Intersection assignment by also drawing the reflected rays"
        if (visualDebug.shadingAndIntersection && visualDebug.recursiveRayTracer) {
            drawRay(ray, colour);
        }
        // Visual Debugger 3.6
        if (visualDebug.interpolatedNormal) {
            drawRay(Ray(hitInfo.v0.position, hitInfo.v0.normal, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            drawRay(Ray(hitInfo.v1.position, hitInfo.v1.normal, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            drawRay(Ray(hitInfo.v2.position, hitInfo.v2.normal, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            drawRay(Ray(hitInfo.intersectionPoint, hitInfo.interpolatedNormal, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            drawRay(Ray(hitInfo.intersectionPoint, hitInfo.planeNormal, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        }
        return colour;
    } else {
        if (visualDebug.shadingAndIntersection || visualDebug.recursiveRayTracer) {
            // Draw a red debug ray if the ray missed.
            drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        }
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}

static void setOpenGLMatrices(const Trackball& camera);
static void drawLightsOpenGL(const Scene& scene, const Trackball& camera, int selectedLight);
static void drawSceneOpenGL(const Scene& scene);

// This is the main rendering function. You are free to change this function in any way (including the function signature).
static void renderRayTracing(const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen, const VisualDebug& visualDebug)
{
#ifndef NDEBUG
    // Single threaded in debug mode
    //bvh.loadVertexNormals();
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos{
                float(x) / windowResolution.x * 2.0f - 1.0f,
                float(y) / windowResolution.y * 2.0f - 1.0f
            };
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            glm::vec3 colour = getFinalColor(scene, bvh, cameraRay, visualDebug);
            float numberOfColours = 1;
            if (visualDebug.multipleRaysPerPixel) {
                std::vector<float> samplesForX;
                std::vector<float> samplesForY;
                srand(duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
                for (int i = 0; i < 4; i++) {
                    samplesForX.push_back(
                        ((-0.5f + ((float)rand() / RAND_MAX)) / 1000.0f) / windowResolution.x * 2.0f);
                    samplesForY.push_back(
                        ((-0.5f + ((float)rand() / RAND_MAX)) / 1000.0f) / windowResolution.y * 2.0f);
                }
            std:
                shuffle(std::begin(samplesForX), std::end(samplesForX), std::default_random_engine{});
                for (size_t i = 0; i < 4; i++) {
                    const glm::vec2 samplePixelPos{
                            glm::clamp(normalizedPixelPos.x + samplesForX.at(i), -1.0f, 1.0f),
                            glm::clamp(normalizedPixelPos.y + samplesForY.at(i), -1.0f, 1.0f)
                    };
                    const Ray sampleRay = camera.generateRay(samplePixelPos);
                    colour += getFinalColor(scene, bvh, sampleRay, visualDebug);
                    numberOfColours++;
                }
            }
            screen.setPixel(x, y, colour / numberOfColours);
        }
    }
#else
    // Multi-threaded in release mode
    const tbb::blocked_range2d<int, int> windowRange{ 0, windowResolution.y, 0, windowResolution.x };
    //bvh.loadVertexNormals();
    tbb::parallel_for(windowRange, [&](tbb::blocked_range2d<int, int> localRange) {
        for (int y = std::begin(localRange.rows()); y != std::end(localRange.rows()); y++) {
            for (int x = std::begin(localRange.cols()); x != std::end(localRange.cols()); x++) {
                // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
                const glm::vec2 normalizedPixelPos{
                    float(x) / windowResolution.x * 2.0f - 1.0f,
                    float(y) / windowResolution.y * 2.0f - 1.0f
                };
                const Ray cameraRay = camera.generateRay(normalizedPixelPos);
                glm::vec3 colour = getFinalColor(scene, bvh, cameraRay, visualDebug);
                float numberOfColours = 1.0f;
                // If multipleRaysPerPixel per pixel is enabled create a random sample, otherwise consider only the original ray
                if (visualDebug.multipleRaysPerPixel) {
                    std::vector<float> samplesForX;
                    std::vector<float> samplesForY;
                    srand(duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count());
                    for (int i = 0; i < 19; i++) {
                        samplesForX.push_back(
                            ((-0.5f + ((float)rand() / RAND_MAX)) / 1000.0f) / windowResolution.x * 2.0f);
                        samplesForY.push_back(
                            ((-0.5f + ((float)rand() / RAND_MAX)) / 1000.0f) / windowResolution.y * 2.0f);
                    }
                    std::shuffle(std::begin(samplesForX), std::end(samplesForX), std::default_random_engine{});
                    for (size_t i = 0; i < 19; i++) {
                        const glm::vec2 samplePixelPos{
                                glm::clamp(normalizedPixelPos.x + samplesForX.at(i), -1.0f, 1.0f),
                                glm::clamp(normalizedPixelPos.y + samplesForY.at(i), -1.0f, 1.0f)
                        };
                        const Ray sampleRay = camera.generateRay(samplePixelPos);
                        colour += getFinalColor(scene, bvh, sampleRay, visualDebug);
                        numberOfColours += 1.0f;
                    }
                }
                screen.setPixel(x, y, (colour / numberOfColours));
            }
        }
        });
#endif
    if (visualDebug.bloomOn) {
        bloom(scene, camera, bvh, screen, visualDebug, 8);
    }
}

int main(int argc, char** argv)
{
    Trackball::printHelp();
    std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
        << std::endl;

    Window window{ "Final Project", windowResolution, OpenGLVersion::GL2 };
    Screen screen{ windowResolution };
    Trackball camera{ &window, glm::radians(50.0f), 3.0f };
    camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

    SceneType sceneType{ SceneType::SingleTriangle };
    std::optional<Ray> optDebugRay;
    std::vector<Ray> sampleRays;
    Scene scene = loadScene(sceneType, dataPath);
    BoundingVolumeHierarchy bvh{ &scene };

    int bvhDebugLevel = 0;
    int bins = 2;
    bool debugBVH{ false };
    bool changebins{ false };

    // Added by Bram
    int bvhLeafIndex = 0;
    bool drawLeafs{ false };
    ViewMode viewMode { ViewMode::Rasterization };
    //Added by Bolek
    VisualDebug visualDebug = {};
    bool shadingAndIntersection { false };
    bool recursiveRayTracer { false };
    bool TraversalAdvancedSelected{ false };
    bool TraversalBasic { false };
    bool TraversalAdvanced { false };
    bool Transparency { false };
    bool transparency { false };
    bool hardShadows { false };
    bool segmentLightSourceShadows { false };
    bool parallelogramLightSourceShadows { false };
    bool interpolatedNormal { false };
    bool glossyReflections { false };
    bool multipleRaysPerPixel { false };
    bool bloomOn{ false };
    bool notVisitedNodesTraversal{ false };
    bool bloomDebug{ false };
    int bloomStage = 0;
    bool drawIntersectedButNotVisited{ false };

    window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
        if (action == GLFW_PRESS) {
            switch (key) {
            case GLFW_KEY_R: {
                // Shoot a ray. Produce a ray from camera to the far plane.
                const auto tmp = window.getNormalizedCursorPos();
                sampleRays.clear();
                // Same as for ray-traced mode. If multipleRaysPerPixel create a random sample otherwise use only the original ray
                if (visualDebug.multipleRaysPerPixel) {
                    std::cout << "CURSOR_POS: " << tmp.x << ", " << tmp.y << std::endl;
                    std::vector<float> samplesForX;
                    std::vector<float> samplesForY;
                    srand(duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count());
                    for (int i = 0; i < 10; i++) {
                        samplesForX.push_back((-0.5f + ((float) rand() / RAND_MAX)) / 1000.0f);
                        samplesForY.push_back((-0.5f + ((float) rand() / RAND_MAX)) / 1000.0f);
                    }
                    std:
                    shuffle(std::begin(samplesForX), std::end(samplesForX), std::default_random_engine{});
                    for (size_t i = 0; i < 10; i++) {
                        const glm::vec2 samplePixelPos{
                                glm::clamp(float(tmp.x) + samplesForX.at(i), -1.0f, 1.0f),
                                glm::clamp(float(tmp.y) + samplesForY.at(i), -1.0f, 1.0f)
                        };
                        std::cout << samplePixelPos.x << ", " << samplePixelPos.y << std::endl;
                        const Ray sampleRay = camera.generateRay(samplePixelPos * 2.0f - 1.0f);
                        sampleRays.push_back(sampleRay);
                    }
                }
                // Original ray
                optDebugRay = camera.generateRay(tmp * 2.0f - 1.0f);
            } break;
            case GLFW_KEY_ESCAPE: {
                window.close();
            } break;
            };
        }
    });

    int selectedLightIdx = scene.lights.empty() ? -1 : 0;
    while (!window.shouldClose()) {
        window.updateInput();

        // === Setup the UI ===
        ImGui::Begin("Final Project");
        {
            constexpr std::array items { "SingleTriangle", "Cube (segment light)", "Cornell Box (with mirror)", "Cornell Box (parallelogram light and mirror)", "Monkey", "Teapot", "Dragon", /* "AABBs",*/ "Spheres", /*"Mixed",*/ "Custom" };
            if (ImGui::Combo("Scenes", reinterpret_cast<int*>(&sceneType), items.data(), int(items.size()))) {
                optDebugRay.reset();
                scene = loadScene(sceneType, dataPath);
                selectedLightIdx = scene.lights.empty() ? -1 : 0;
                bvh = BoundingVolumeHierarchy(&scene);
                if (optDebugRay) {
                    HitInfo dummy {};
                    bvh.intersect(*optDebugRay, dummy, visualDebug);
                    //for (size_t i = 0; i < sampleRays.size(); i++)
                    //{
                    //    bvh.intersect(sampleRays.at(i), dummy);
                    //}
                }
            }
        }
        {
            constexpr std::array items { "Rasterization", "Ray Traced" };
            ImGui::Combo("View mode", reinterpret_cast<int*>(&viewMode), items.data(), int(items.size()));
        }
        if (ImGui::Button("Render to file")) {
            // Show a file picker.
            nfdchar_t* pOutPath = nullptr;
            const nfdresult_t result = NFD_SaveDialog("bmp", nullptr, &pOutPath);
            if (result == NFD_OKAY) {
                std::filesystem::path outPath { pOutPath };
                free(pOutPath); // NFD is a C API so we have to manually free the memory it allocated.
                outPath.replace_extension("bmp"); // Make sure that the file extension is *.bmp

                // Perform a new render and measure the time it took to generate the image.
                using clock = std::chrono::high_resolution_clock;
                const auto start = clock::now();
                renderRayTracing(scene, camera, bvh, screen, visualDebug);
                const auto end = clock::now();
                std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;

                // Store the new image.
                screen.writeBitmapToFile(outPath);
            }
        }

        // Checkboxes in the GUI that allow you to enable/disable certain (extra) features
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Enable/Disable Features");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Glossy Reflections", &glossyReflections);
            visualDebug.glossyReflections = glossyReflections;
        }

        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Multiple Rays Per Pixel", &multipleRaysPerPixel);
            visualDebug.multipleRaysPerPixel = multipleRaysPerPixel;
        }

        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Transparency", &transparency);
            visualDebug.transparency = transparency;
        }

        //Added By Bolek - Bloom on/off
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Bloom on", &bloomOn);
            visualDebug.bloomOn = bloomOn;
        }

        //Added By Bolek - Travesal Selection
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Select Advanced Traversal");
        ImGui::Text("Traversal Basic, unless this option is ticked");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Advanced", &TraversalAdvancedSelected);
            visualDebug.TraversalAdvancedSelected = TraversalAdvancedSelected;
        }

        // Checkboxes in the GUI that allow you to enable/disable the debuggers for the features
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw BVH", &debugBVH);
            if (debugBVH) 
                ImGui::SliderInt("BVH Level", &bvhDebugLevel, 0, bvh.numLevels() - 1);
        }


        // Added By Bram - Cycle through all leaf nodes
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Cycle through leaf nodes", &drawLeafs);
            if (drawLeafs) {
                ImGui::SliderInt("Leaf node", &bvhLeafIndex, 0, bvh.numLeafNodes()-1);
                if (ImGui::Button("<") && bvhLeafIndex > 0) {
                    bvhLeafIndex -= 1;
                }
                if (ImGui::Button(">") && bvhLeafIndex < bvh.numLeafNodes()-1) {
                    bvhLeafIndex += 1;
                }
            }
        }

        // Added By Bram - Change the amount of bins for SAH
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Choose amount of bins for SAH", &changebins);
            if (changebins) {
                ImGui::InputInt("Number of bins", &bins);
                bvh.setNumBins(bins);
                if (ImGui::Button("load again")) {
                    SceneType type = sceneType;
                    optDebugRay.reset();
                    scene = loadScene(SingleTriangle, dataPath);
                    selectedLightIdx = scene.lights.empty() ? -1 : 0;
                    bvh = BoundingVolumeHierarchy(&scene);
                    if (optDebugRay) {
                        HitInfo dummy{};
                        bvh.intersect(*optDebugRay, dummy, visualDebug);
                        //for (size_t i = 0; i < sampleRays.size(); i++)
                        //{
                        //    bvh.intersect(sampleRays.at(i), dummy);
                        //}
                    }
                    optDebugRay.reset();
                    scene = loadScene(type, dataPath);
                    selectedLightIdx = scene.lights.empty() ? -1 : 0;
                    bvh = BoundingVolumeHierarchy(&scene);
                    if (optDebugRay) {
                        HitInfo dummy{};
                        bvh.intersect(*optDebugRay, dummy, visualDebug);
                        //for (size_t i = 0; i < sampleRays.size(); i++)
                        //{
                        //    bvh.intersect(sampleRays.at(i), dummy);
                        //}
                    }
                }
            }
        }

        //Added By Bolek - Phong Shading Bool
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Basic Shading Ray", &shadingAndIntersection);
            visualDebug.shadingAndIntersection = shadingAndIntersection;
        }
        //Added By Bolek - Recursive RayTracer Bool
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Recursive Ray", &recursiveRayTracer);
            visualDebug.recursiveRayTracer = recursiveRayTracer;
        }
        //Added By Bolek - Basic Traversal
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Basic Traversal", &TraversalBasic);
            visualDebug.TraversalBasic = TraversalBasic;
        }
        //Added By Bolek - Advanced Traversal
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Advanced Traversal", &TraversalAdvanced);
            visualDebug.TraversalAdvanced = TraversalAdvanced;
        }

        //Added By Bolek - draw visited, but not intersected Nodes
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw intersected, but not visited", &drawIntersectedButNotVisited);
            visualDebug.drawIntersectedButNotVisited = drawIntersectedButNotVisited;
        }

        //Added By Bolek - draw not visited Nodes
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw not visited Nodes", &notVisitedNodesTraversal);
            visualDebug.notVisitedNodesTraversal = notVisitedNodesTraversal;
        }

        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Hard Shadows", &hardShadows);
            visualDebug.hardShadows = hardShadows;
        }

        //Added By Bolek - Transparency
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Transparency", &Transparency);
            visualDebug.Transparency = Transparency;
        }

        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Segment Light Source Shadows", &segmentLightSourceShadows);
            visualDebug.segmentLightSourceShadows = segmentLightSourceShadows;
        }

        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Parallelogram Light Source Shadows", &parallelogramLightSourceShadows);
            visualDebug.parallelogramLightSourceShadows = parallelogramLightSourceShadows;
        }

        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw Interpolated Normal", &interpolatedNormal);
            visualDebug.interpolatedNormal = interpolatedNormal;
        }

        //Added By Bolek - Bloom incl. stage selection
        if (viewMode == ViewMode::Rasterization && bloomOn) {
            ImGui::Checkbox("Draw Bloom", &bloomDebug);
            if (&bloomDebug) {
                ImGui::SliderInt("bloom stage", &bloomStage, 1, 2);
                visualDebug.bloomStage = bloomStage;
            }
            visualDebug.bloomDebug = bloomDebug;
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Lights");
        {
            std::vector<std::string> options;
            options.push_back("None");
            for (size_t i = 0; i < scene.lights.size(); i++) {
                options.push_back("Light " + std::to_string(i));
            }
            std::vector<const char*> optionsPointers;
            std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers), [](const auto& str) { return str.c_str(); });

            // Offset such that selectedLightIdx=-1 becomes item 0 (None).
            ++selectedLightIdx;
            ImGui::Combo("Selected light", &selectedLightIdx, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
            --selectedLightIdx;

            if (selectedLightIdx >= 0) {
                setOpenGLMatrices(camera);
                std::visit(
                    make_visitor(
                        [&](PointLight& light) {
                            showImGuizmoTranslation(window, camera, light.position); // 3D controls to translate light source.
                            ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                            ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                        },
                        [&](SegmentLight& light) {
                            static int selectedEndpoint = 0;
                            // 3D controls to translate light source.
                            if (selectedEndpoint == 0)
                                showImGuizmoTranslation(window, camera, light.endpoint0);
                            else
                                showImGuizmoTranslation(window, camera, light.endpoint1);

                            const std::array<const char*, 2> endpointOptions { "Endpoint 0", "Endpoint 1" };
                            ImGui::Combo("Selected endpoint", &selectedEndpoint, endpointOptions.data(), (int)endpointOptions.size());
                            ImGui::DragFloat3("Endpoint 0", glm::value_ptr(light.endpoint0), 0.01f, -3.0f, 3.0f);
                            ImGui::DragFloat3("Endpoint 1", glm::value_ptr(light.endpoint1), 0.01f, -3.0f, 3.0f);
                            ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                            ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                        },
                        [&](ParallelogramLight& light) {
                            glm::vec3 vertex1 = light.v0 + light.edge01;
                            glm::vec3 vertex2 = light.v0 + light.edge02;

                            static int selectedVertex = 0;
                            // 3D controls to translate light source.
                            if (selectedVertex == 0)
                                showImGuizmoTranslation(window, camera, light.v0);
                            else if (selectedVertex == 1)
                                showImGuizmoTranslation(window, camera, vertex1);
                            else
                                showImGuizmoTranslation(window, camera, vertex2);

                            const std::array<const char*, 3> vertexOptions { "Vertex 0", "Vertex 1", "Vertex 2" };
                            ImGui::Combo("Selected vertex", &selectedVertex, vertexOptions.data(), (int)vertexOptions.size());
                            ImGui::DragFloat3("Vertex 0", glm::value_ptr(light.v0), 0.01f, -3.0f, 3.0f);
                            ImGui::DragFloat3("Vertex 1", glm::value_ptr(vertex1), 0.01f, -3.0f, 3.0f);
                            light.edge01 = vertex1 - light.v0;
                            ImGui::DragFloat3("Vertex 2", glm::value_ptr(vertex2), 0.01f, -3.0f, 3.0f);
                            light.edge02 = vertex2 - light.v0;

                            ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                            ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                            ImGui::ColorEdit3("Color 2", glm::value_ptr(light.color2));
                            ImGui::ColorEdit3("Color 3", glm::value_ptr(light.color3));
                        },
                        [](auto) { /* any other type of light */ }),
                    scene.lights[selectedLightIdx]);
            }
        }

        if (ImGui::Button("Add point light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(PointLight { .position = glm::vec3(0.0f), .color = glm::vec3(1.0f) });
        }
        if (ImGui::Button("Add segment light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(SegmentLight { .endpoint0 = glm::vec3(0.0f), .endpoint1 = glm::vec3(1.0f), .color0 = glm::vec3(1, 0, 0), .color1 = glm::vec3(0, 0, 1) });
        }
        if (ImGui::Button("Add parallelogram light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(ParallelogramLight {
                .v0 = glm::vec3(0.0f),
                .edge01 = glm::vec3(1, 0, 0),
                .edge02 = glm::vec3(0, 1, 0),
                .color0 = glm::vec3(1, 0, 0), // red
                .color1 = glm::vec3(0, 1, 0), // green
                .color2 = glm::vec3(0, 0, 1), // blue
                .color3 = glm::vec3(1, 1, 1) // white
            });
        }
        if (selectedLightIdx >= 0 && ImGui::Button("Remove selected light")) {
            scene.lights.erase(std::begin(scene.lights) + selectedLightIdx);
            selectedLightIdx = -1;
        }

        // Clear screen.
        glViewport(0, 0, window.getFrameBufferSize().x, window.getFrameBufferSize().y);
        glClearDepth(1.0f);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        setOpenGLMatrices(camera);

        // Draw either using OpenGL (rasterization) or the ray tracing function.
        switch (viewMode) {
        case ViewMode::Rasterization: {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            drawSceneOpenGL(scene);
            if (optDebugRay) {
                // Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
                // draw the rays instead.
                enableDrawRay = true;
                glDisable(GL_LIGHTING);
                glDepthFunc(GL_LEQUAL);
                (void)getFinalColor(scene, bvh, *optDebugRay, visualDebug);
                // Draw the sample rays. If multipleRaysPerPixel is not enable then sampleRays.size() returns 0 and no extra rays are drawned
                for (size_t i = 0; i < sampleRays.size(); i++)
                {
                    (void)getFinalColor(scene, bvh, sampleRays.at(i), visualDebug);
                }
                enableDrawRay = false;
            }
            glPopAttrib();
        } break;
        case ViewMode::RayTracing: {
            screen.clear(glm::vec3(0.0f));
            renderRayTracing(scene, camera, bvh, screen, visualDebug);
            screen.setPixel(0, 0, glm::vec3(1.0f));
            screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
        } break;
        default:
            break;
        };

        drawLightsOpenGL(scene, camera, selectedLightIdx);

        if (debugBVH) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            enableDrawRay = true;
            bvh.debugDraw(bvhDebugLevel);
            enableDrawRay = false;
            glPopAttrib();
        }

        // Added by Bram
        if (drawLeafs) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            enableDrawRay = true;
            bvh.drawLeafs(bvhLeafIndex);
            enableDrawRay = false;
            glPopAttrib();
        }

        ImGui::End();
        window.swapBuffers();
    }

    return 0;
}

static void setOpenGLMatrices(const Trackball& camera)
{
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void drawLightsOpenGL(const Scene& scene, const Trackball& camera, int selectedLight)
{
    // Normals will be normalized in the graphics pipeline.
    glEnable(GL_NORMALIZE);
    // Activate rendering modes.
    glEnable(GL_DEPTH_TEST);
    // Draw front and back facing triangles filled.
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    // Interpolate vertex colors over the triangles.
    glShadeModel(GL_SMOOTH);

    glDisable(GL_LIGHTING);
    // Draw all non-selected lights.
    for (size_t i = 0; i < scene.lights.size(); i++) {
        std::visit(
            make_visitor(
                [](const PointLight& light) { drawSphere(light.position, 0.01f, light.color); },
                [](const SegmentLight& light) {
                    glPushAttrib(GL_ALL_ATTRIB_BITS);
                    glBegin(GL_LINES);
                    glColor3fv(glm::value_ptr(light.color0));
                    glVertex3fv(glm::value_ptr(light.endpoint0));
                    glColor3fv(glm::value_ptr(light.color1));
                    glVertex3fv(glm::value_ptr(light.endpoint1));
                    glEnd();
                    glPopAttrib();
                    drawSphere(light.endpoint0, 0.01f, light.color0);
                    drawSphere(light.endpoint1, 0.01f, light.color1);
                },
                [](const ParallelogramLight& light) {
                    glPushAttrib(GL_ALL_ATTRIB_BITS);
                    glBegin(GL_QUADS);
                    glColor3fv(glm::value_ptr(light.color0));
                    glVertex3fv(glm::value_ptr(light.v0));
                    glColor3fv(glm::value_ptr(light.color1));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge01));
                    glColor3fv(glm::value_ptr(light.color3));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge01 + light.edge02));
                    glColor3fv(glm::value_ptr(light.color2));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge02));
                    glEnd();
                    glPopAttrib();
                },
                [](auto) { /* any other type of light */ }),
            scene.lights[i]);
    }

    // Draw a colored sphere at the location at which the trackball is looking/rotating around.
    glDisable(GL_LIGHTING);
    drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}

void drawSceneOpenGL(const Scene& scene)
{
    // Activate the light in the legacy OpenGL mode.
    glEnable(GL_LIGHTING);

    // Tell OpenGL where the lights are (so it nows how to shade surfaces in the scene).
    // This is only used in the rasterization view. OpenGL only supports point lights so
    // we replace segment/parallelogram lights by point lights.
    int i = 0;
    const auto enableLight = [&](const glm::vec3& position, const glm::vec3 color) {
        glEnable(GL_LIGHT0 + i);
        const glm::vec4 position4 { position, 1 };
        glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
        const glm::vec4 color4 { glm::clamp(color, 0.0f, 1.0f), 1.0f };
        const glm::vec4 zero4 { 0.0f, 0.0f, 0.0f, 1.0f };
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
        // NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
        // The distance is not in world space but in NDC space!
        glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
        i++;
    };
    for (const auto& light : scene.lights) {
        std::visit(
            make_visitor(
                [&](const PointLight& light) {
                    enableLight(light.position, light.color);
                },
                [&](const SegmentLight& light) {
                    // Approximate with two point lights: one at each endpoint.
                    enableLight(light.endpoint0, 0.5f * light.color0);
                    enableLight(light.endpoint1, 0.5f * light.color1);
                },
                [&](const ParallelogramLight& light) {
                    enableLight(light.v0, 0.25f * light.color0);
                    enableLight(light.v0 + light.edge01, 0.25f * light.color1);
                    enableLight(light.v0 + light.edge02, 0.25f * light.color2);
                    enableLight(light.v0 + light.edge01 + light.edge02, 0.25f * light.color3);
                },
                [](auto) { /* any other type of light */ }),
            light);
    }

    // Draw the scene and the ray (if any).
    drawScene(scene);
}

//Code for the transparency extra feature
glm::vec3 transparency(const Material& shadingData, const glm::vec3& intersectionPoint, const glm::vec3 color, Ray ray, HitInfo& hitInfo, const Scene& scene, const BoundingVolumeHierarchy& bvh, const VisualDebug& visualDebug) {
    //Base case for opaque object
    if (shadingData.transparency == 1.0f) {
        return color;
    }
    //Get ray that shoots after the first object is hit (in the same direction)
    Ray continuedRay = Ray(intersectionPoint + (0.0001f * ray.direction), ray.direction, std::numeric_limits<float>::max());
    //Get the color of object (or void) behind first object
    glm::vec3 secondColor = getFinalColor(scene, bvh, continuedRay, visualDebug);

    //Visual Debug for transparency
    //Draws a ray, if a transparent object is hit with the combined color of said object
    if (visualDebug.Transparency) {
        drawRay(ray, (shadingData.transparency) * color + (1 - shadingData.transparency) * secondColor);
    }

    //Mix the two based on the transparency attribute
    return (shadingData.transparency) * color + (1 - shadingData.transparency) * secondColor;

}

//Box filter by Bolek
glm::vec3 boxFilter(int filterSize, float pixelX, float pixelY, const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen, const VisualDebug& visualDebug)
{
    //Start with a sum of zero
    glm::vec3 sum = glm::vec3{ 0.0f };
    //go over each neigboring pixel of the selected pixel
    for (int y = -filterSize; y < filterSize + 1; y++) {
        for (int x = -filterSize; x != filterSize + 1; x++) {

            //If you can access neighbouring pixel, add its values to sum
            glm::vec3 colour;
            if (((pixelX + x) < 0) || ((pixelY + y) < 0)) {
                colour = glm::vec3{0.0f, 0.0f, 0.0f};
            }
            else {
                colour = screen.getBloom(pixelX + x, pixelY + y);
            }

            sum += colour;
        }

        //Take the average
        sum /= ((2 * filterSize + 1) * (2 * filterSize + 1)); 

        //Return sum
        return sum;

    }   
}
       
    //bloom done by Bolek
    void bloom(const Scene & scene, const Trackball & camera, const BoundingVolumeHierarchy & bvh, Screen & screen, const VisualDebug & visualDebug, int filterSize)
    {
        //Default color black
        glm::vec3 colorOld = glm::vec3{ 0.0f, 0.0f, 0.0f };
        //Go over each pixel
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {

                //Get color of pixel
                colorOld = screen.getPixel(x, y);

                //In case of a color channel having a high value, do boxfilter
                if ((colorOld.x > 0.65f) || (colorOld.y > 0.65f) || (colorOld.z > 0.65f)) {
                    screen.setBloom(x, y, colorOld);

                    //Visual Debug Bloom
                    //If stage 1 is selected in the GUI, we render all colors that qualify for box filter (result is black and white picture)
                    if (visualDebug.bloomDebug && visualDebug.bloomStage == 1) {
                        screen.setPixel(x, y, colorOld);
                    }
                }
                else {
                    screen.setBloom(x, y, glm::vec3{ 0.0f, 0.0f, 0.0f });
                    //Visual Debug Bloom
                    //If stage 1 is selected in the GUI, we render all colors that don't qualify for box filter (result is black and white picture)
                    if (visualDebug.bloomDebug && visualDebug.bloomStage == 1) {
                        screen.setPixel(x, y, glm::vec3{ 0.0f, 0.0f, 0.0f });
                    }
                }
            }
        }
        //Visual Debug Bloom
        //If stage 1 is selected in the GUI, this is where we stop our code to render the black and white image
        if (visualDebug.bloomDebug && visualDebug.bloomStage == 1) {
            return;
        }

        //Go over each pixel again
        for (int k = 0;k < windowResolution.y; k++) {
            for (int h = 0; h != windowResolution.x; h++) {
                    //apply box filter to all pixels
                    glm::vec3 colorNew = boxFilter(filterSize, k, h, scene, camera, bvh, screen, visualDebug);
                    //get the current color
                    glm::vec3 colorDefault = screen.getPixel(k, h);
                    //overlay the new color on top of the current one to get bloom effect
                    screen.setPixel(k, h, colorNew * glm::vec3(1.7f) + colorDefault);
                    //Visual Debug Bloom
                    //If stage 2 is selected in the GUI, we render an image of the boxfilter (only the new color)
                    if (visualDebug.bloomDebug && visualDebug.bloomStage == 2) {
                        screen.setPixel(k, h, colorNew);
                    }
            }
        }
    }




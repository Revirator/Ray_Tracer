#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <set>

Scene* scene;
int axis;

// Class of self made comparator, which compares two vector<int>'s
// where the value at index 0 is the mesh index, and the value at index 1 is the triangle index inside that mesh
class compare {
public:
    bool operator()(const std::vector<int>& i,const std::vector<int>& j) {
        std::vector<float> leftCentroid;
        std::vector<float> rightCentroid;

        // get mesh index of first triangle
        int meshi = i[0];
        // get mesh index of second triangle
        int meshj = j[0];
        // get triangle index of first triangle
        int indexi = i[1];
        // get triangle index of second triangle
        int indexj = j[1];

        leftCentroid.push_back((scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).x).position.x + scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).y).position.x + scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).z).position.x) / 3.0f);
        leftCentroid.push_back((scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).x).position.y + scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).y).position.y + scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).z).position.y) / 3.0f);
        leftCentroid.push_back((scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).x).position.z + scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).y).position.z + scene->meshes.at(meshi).vertices.at(scene->meshes.at(meshi).triangles.at(indexi).z).position.z) / 3.0f);

        rightCentroid.push_back((scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).x).position.x + scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).y).position.x + scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).z).position.x) / 3.0f);
        rightCentroid.push_back((scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).x).position.y + scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).y).position.y + scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).z).position.y) / 3.0f);
        rightCentroid.push_back((scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).x).position.z + scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).y).position.z + scene->meshes.at(meshj).vertices.at(scene->meshes.at(meshj).triangles.at(indexj).z).position.z) / 3.0f);
        
        // compare on the coordinate of the triangles's centroid, variable 'axis' determines which coordinate to look at.
        if (leftCentroid[axis] < rightCentroid[axis]) {
            return true;
        }
        return false;
    }
};
// all the nodes in the bvh tree stored in a vector, 
// for each parent at index a, it holds that left and right children have indices 2*a + 1 and 2*a + 2 respectfully
std::vector<BoundingVolumeHierarchy::Node> nodes; 
int depth_BVH; // keep track of depth of tree
int num_leafs;
int numbins;

// takes a vector of triangle indices and returns the aabb that wraps around those triangles
AxisAlignedBox getBox(std::vector<std::vector<int>> indices) {
    float lowx = INT_MAX;
    float lowy = INT_MAX;
    float lowz = INT_MAX;
    float upx = INT_MIN;
    float upy = INT_MIN;
    float upz = INT_MIN;
    AxisAlignedBox box;

    // loop through all indices
    for (std::vector<int>& i : indices) {

        int& mesh = i[0]; // get the mesh index
        int& index = i[1]; // and get triangle index

        // get the corner vertices of the triangle
        Vertex& v0 = scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).x);
        Vertex& v1 = scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).y);
        Vertex& v2 = scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).z);
        
        // compare all coordinates, to get both the lowest and highest value of all triangles in the loop
        lowx = std::fminf(std::fminf(std::fminf(v1.position.x, v0.position.x), v2.position.x), lowx);
        lowy = std::fminf(std::fminf(std::fminf(v1.position.y, v0.position.y), v2.position.y), lowy);
        lowz = std::fminf(std::fminf(std::fminf(v1.position.z, v0.position.z), v1.position.z), lowz);
        upx = std::fmaxf(std::fmaxf(std::fmaxf(v1.position.x, v0.position.x), v2.position.x), upx);
        upy = std::fmaxf(std::fmaxf(std::fmaxf(v1.position.y, v0.position.y), v2.position.y), upy);
        upz = std::fmaxf(std::fmaxf(std::fmaxf(v1.position.z, v0.position.z), v2.position.z), upz);
    }
    return { glm::vec3(lowx, lowy, lowz), glm::vec3(upx, upy, upz) };
}
// recursively builds the tree, param node is the 'current node', level is the level of the current node, 
// indices is a vector that holds indices for both the mesh and the triangle, per triangle that is present in the current node's box.
// Indices is already sorted on triangle centroids, with looking at the right axis belonging to current node's level.
void buildBVH(BoundingVolumeHierarchy::Node node, int maxLevel, int level, std::vector<std::vector<int>> indices) {

    // make sure we remember the biggest level
    if (level > depth_BVH) {
        depth_BVH = level;
    }

    // if indices.size() > 1, current node is not a leaf, so we create children and call this method again for both children.
    if (indices.size() > 1 && maxLevel > level) {

        // set isLeaf to false
        node.isLeaf = false;

        // create both children
        BoundingVolumeHierarchy::Node child1;
        BoundingVolumeHierarchy::Node child2;
        child1.level = level + 1;
        child2.level = level + 1;

        // create a new vector of indices for each child, to use as a parameter for the childs's call of this method
        std::vector<std::vector<int>> indices1;
        std::vector<std::vector<int>> indices2;

        // since 'indices' was already sorted properly, we can put the left half into indices1, and else into indices2
        float split = indices.size() / 2.0f;
        for (int i = 0; i < indices.size(); i++) {
            if (i < split) {
                indices1.push_back(indices.at(i));
            }
            else {
                indices2.push_back(indices.at(i));
            }
        }
        // get the correct axisAlignedBox for both children
        child1.box = getBox(indices1);
        child2.box = getBox(indices2);

        // get comparator
        compare c;

        // axis alternates, so set new axis
        axis = (level + 1) % 3;

        // sort both indices1 and indices2
        std::sort(indices1.begin(), indices1.end(), c);
        std::sort(indices2.begin(), indices2.end(), c);

        // recursively repeat for both children
        buildBVH(child1, maxLevel, level + 1, indices1);
        std::vector<int> vector1;
        vector1.push_back(nodes.size()-1);
        node.indices.push_back(vector1);

        buildBVH(child2, maxLevel, level + 1, indices2);
        std::vector<int> vector2;
        vector2.push_back(nodes.size()-1);
        node.indices.push_back(vector2);

        nodes.push_back(node);
    }
    // We have only one triangle in the box, so the current node is a leaf node
    else {
        node.isLeaf = true;
        node.box = getBox(indices);
        node.indices = indices;
        nodes.push_back(node);
        num_leafs++;
    }
}
// This method also builds a bvh just like the method above, only now, we dont split at triangle centroid median,
// but we split between predefined bins, for each node we calculate the cost and then choose the option with the lowest cost, 
// this cost checking happens for all axes.
void buildSAH(BoundingVolumeHierarchy::Node node, int maxLevel, int numBins, int level, std::vector<std::vector<int>> indices) {

    // make sure we remember the biggest level
    if (level > depth_BVH) {
        depth_BVH = level;
    }
    // if indices.size() > 1, current node is not a leaf, so we create children and call this method again for both children.
    if (indices.size() > 1 && maxLevel > level) {
        // set isLeaf to false
        node.isLeaf = false;

        // create both children
        BoundingVolumeHierarchy::Node child1;
        BoundingVolumeHierarchy::Node child2;
        child1.level = level + 1;
        child2.level = level + 1;

        // create a new vector of indices for each child, to use as a parameter for the childs's call of this method
        std::vector<std::vector<int>> indices1;
        std::vector<std::vector<int>> indices2;

        float minCost = INT_MAX;
        // do this for all three axis
        for (int splittingAxis = 0; splittingAxis < 3; splittingAxis++) {
            glm::vec2 range;
            if (splittingAxis == 0) {
                range = glm::vec2(node.box.lower.x, node.box.upper.x);
            }
            if (splittingAxis == 1) {
                range = glm::vec2(node.box.lower.y, node.box.upper.y);
            }
            if (splittingAxis == 2) {
                range = glm::vec2(node.box.lower.z, node.box.upper.z);
            }
            // loop for each possible bin
            for (int bin = 1; bin < numBins; bin++) {
                std::vector<std::vector<int>> indices1temp = {};
                std::vector<std::vector<int>> indices2temp = {};
                AxisAlignedBox box1;
                AxisAlignedBox box2;

                for (std::vector<int> i : indices) {

                    // get triangle centroid
                    int mesh = i[0];
                    int index = i[1];
                    std::vector<float> centroid;
                    centroid.push_back((scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).x).position.x + scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).y).position.x + scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).z).position.x) / 3.0f);
                    centroid.push_back((scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).x).position.y + scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).y).position.y + scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).z).position.y) / 3.0f);
                    centroid.push_back((scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).x).position.z + scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).y).position.z + scene->meshes.at(mesh).vertices.at(scene->meshes.at(mesh).triangles.at(index).z).position.z) / 3.0f);

                    // push into the belonging indices list
                    if (centroid[splittingAxis] <= range.x + (range.y - range.x) * (float)bin /(float) numBins) {
                        indices1temp.push_back(i);
                    }
                    else {
                        indices2temp.push_back(i);
                    }
                }
                // we only test for children that have triangles, here we check if we have to continue
                if (indices1temp.size() > 0 && indices2temp.size() > 0) {
                    // calculate the cost
                    box1 = getBox(indices1temp);
                    box2 = getBox(indices2temp);
                    float surArea1 = 2 * (box1.upper.z - box1.lower.z) * (box1.upper.y - box1.upper.y) + 2 * (box1.upper.y - box1.lower.y) * (box1.upper.x - box1.lower.x) + 2 * (box1.upper.z - box1.lower.z) * (box1.upper.x - box1.lower.x);
                    float surArea2 = 2 * (box2.upper.z - box2.lower.z) * (box2.upper.y - box2.upper.y) + 2 * (box2.upper.y - box2.lower.y) * (box2.upper.x - box2.lower.x) + 2 * (box2.upper.z - box2.lower.z) * (box2.upper.x - box2.lower.x);
                    float cost = surArea1 * indices1temp.size() + surArea2 * indices2temp.size();
                    if (cost < minCost) {
                        // in case we found the lowest cost so far, we store all the temporary data
                        minCost = cost;
                        child1.box = box1;
                        child2.box = box2;
                        indices1 = indices1temp;
                        indices2 = indices2temp;
                    }
                }
            }
        }
        // if one of the children contains no triangles, we stop and store the current node as a leaf
        if (indices1.size() < 1 || indices2.size() < 1) {
            node.isLeaf = true;
            node.indices = indices;
            nodes.push_back(node);
            num_leafs++;
        }
        else {
            // recursively repeat for both children
            buildSAH(child1, maxLevel, numBins, level + 1, indices1);
            std::vector<int> vector1;
            vector1.push_back(nodes.size() - 1);
            node.indices.push_back(vector1);

            buildSAH(child2, maxLevel, numBins, level + 1, indices2);
            std::vector<int> vector2;
            vector2.push_back(nodes.size() - 1);
            node.indices.push_back(vector2);

            nodes.push_back(node);
        }
    }
    // Leaf node
    else {
        node.isLeaf = true;
        node.box = getBox(indices);
        node.indices = indices;
        nodes.push_back(node);
        num_leafs++;
    }
}


BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    scene = pScene;
    nodes = {};
    if (!pScene->meshes.empty()) {
        Node root;
        root.level = 0;
        depth_BVH = 0;
        num_leafs = 0;
        int mesh = 0;
        std::vector<std::vector<int>> indices;
        for (Mesh& m : pScene->meshes) {
            for (int i = 0; i < m.triangles.size(); i++) {
                std::vector<int> vector = { mesh, i };
                indices.push_back(vector);
            }
            mesh++;
        }
        int maxlevel = 1000;
        root.box = getBox(indices);
        axis = 0;
        compare c;
        std::sort(indices.begin(), indices.end(), c);
        buildBVH(root, maxlevel, 0, indices);
        //if (numbins == 0 || numbins < 2) {
        //    numbins = 2;
        //}
        //buildSAH(root, maxlevel, numbins, 0, indices);
    }
}

void BoundingVolumeHierarchy::setNumBins(int n) {
    numbins = n;
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display.
int BoundingVolumeHierarchy::numLevels() const
{
    return depth_BVH + 1;
}

// Return the number of leaf nodes
int BoundingVolumeHierarchy::numLeafNodes() const
{
    return num_leafs;
}

// call this method to cycle through all leaf nodes and draw the contained triangle(s)
// param int index specifies which leaf node to draw
void BoundingVolumeHierarchy::drawLeafs(int index)
{
    int count = 0;
    for (BoundingVolumeHierarchy::Node& node : nodes) {
        if (node.isLeaf) {
            if (count == index) {
                drawAABB(getBox(node.indices), DrawMode::Wireframe, glm::vec3(0.05f, 1.0f, 0.05f), 0.5);
                glm::vec3 color = glm::vec3{ 0.3f, 0.9f, 0.1f };
                for (int i = 0; i < node.indices.size(); i++) {
                    const auto& mesh = m_pScene->meshes[node.indices[i][0]];

                    drawTriangleColor(mesh, node.indices[i][1], color);
                    float r = fmod(color.x + 0.05f + color.z * 2 / 3, 1.0f);
                    float g = fmod(color.y + 0.05f + color.y * 1 / 4, 1.0f);
                    float b = fmod(color.z + 0.05f + color.x * 1 / 2, 1.0f);
                    while ((r - 1.0f < 0.15f) && (g - 0.0f < 0.15f) && (b - 0.0f < 0.15f)) {
                        float r = fmod(color.x + 0.05f + color.z * 2 / 3, 1.0f);
                        float g = fmod(color.y + 0.05f + color.y * 1 / 4, 1.0f);
                        float b = fmod(color.z + 0.05f + color.x * 1 / 2, 1.0f);
                    }
                    color = glm::vec3{ r,g,b };
                }
            }
            count++;
        }
    }

}
// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{            
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);
    for (BoundingVolumeHierarchy::Node& node : nodes) {
        if (node.level == level) {
            drawAABB(node.box, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.3);
        }
    }
}


// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, const VisualDebug& visualDebug) const
{

    //This method is only used to enter either the basic or advanced traversal
    //Basic is the default traversal, however if "Advanced" is ticked in the GUI, advanced traversal will be selected
    if (visualDebug.TraversalAdvancedSelected) {
        return this->intersectCustomAdvanced(ray, hitInfo, 0, 0, visualDebug);
    }
    else {
        return this->intersectCustomBasic(ray, hitInfo, 0, nodes.size() - 1, visualDebug, glm::vec3{0.3f, 0.7f, 0.1f});
    }
}

//https://www.geeksforgeeks.org/stl-priority-queue-for-structure-or-class/
//Comparitor I coded to create a min-Priority-Queue of tuples <nodeIndex, distance> based on distance to ray
struct CompareDistance {
    bool operator()(std::tuple<int, float> const& distance1, std::tuple<int, float> const& distance2)
    {
        //return true if distance1 is bigger than distance2
        if (get<1>(distance1) > get<1>(distance2)) {
            return true;
        }
        else {
            return false;
        }
    }
};

//Basic traversal
//hitNodes is here to document visited nodes (used for drawing not visited nodes; extra thing I coded)
std::set<int> hitNodes;
bool BoundingVolumeHierarchy::intersectCustomBasic(Ray& ray, HitInfo& hitInfo, float depth, int nodeIndex, const VisualDebug& visualDebug, glm::vec3 recursiveColor) const
{
    bool hit = false;
    //access current node
    if (nodes.size() > 0) {
        Node current = nodes[nodeIndex];

        //check if ray hits AABB
        float t = ray.t;
        bool rayAABB = intersectRayWithShape(current.box, ray);
        ray.t = t;

        //If it hits enter
        if (rayAABB) {
            //Visual Debug 3.5 Traversal Basic
            //In case we wish to "Create an option to draw the intersected but not visited nodes with another color", we access this (Unvisited Node)
            if (visualDebug.TraversalBasic && visualDebug.drawIntersectedButNotVisited && !current.isLeaf) {
                drawAABB(current.box, DrawMode::Wireframe, glm::vec3{1.0f, 0.0f, 0.0f}, 1);
            }
            //In case we wish to "When performing recursive ray-tracing, visualize each recursion at a time", we access this
            if (visualDebug.TraversalBasic && !visualDebug.drawIntersectedButNotVisited) {
                //Draws the current box in a somewhat random color
                drawAABB(current.box, DrawMode::Wireframe, recursiveColor, 1);
                float r = fmod(recursiveColor.x + 0.05f + recursiveColor.z * 2/3, 1.0f);
                float g = fmod(recursiveColor.y + 0.05f + recursiveColor.y * 1/4, 1.0f);
                float b = fmod(recursiveColor.z + 0.05f + recursiveColor.x * 1/2, 1.0f);
                while ((r - 1.0f < 0.15f) && (g - 0.0f < 0.15f) && (b - 0.0f < 0.15f)) {
                    float r = fmod(recursiveColor.x + 0.05f + recursiveColor.z * 2 / 3, 1.0f);
                    float g = fmod(recursiveColor.y + 0.05f + recursiveColor.y * 1 / 4, 1.0f);
                    float b = fmod(recursiveColor.z + 0.05f + recursiveColor.x * 1 / 2, 1.0f);
                }
                recursiveColor = glm::vec3{ r,g,b };
            }
            //Extra thing I coded, adds hit Node's index to hitNodes
            if (visualDebug.notVisitedNodesTraversal) {
                hitNodes.insert(nodeIndex);
            }
            //If node is leaf
            if (current.isLeaf) {
                //Visual Debug 3.5 Traversal Basic
                //In case we wish to "Create an option to draw the intersected but not visited nodes with another color", we access this (visited Node)
                if (visualDebug.TraversalBasic && visualDebug.drawIntersectedButNotVisited) {
                    drawAABB(current.box, DrawMode::Wireframe, glm::vec3{ 0.0f, 1.0f, 0.0f }, 1);
                }
                //Access the meshes that the node is referencing in current.indices[i][0] and build all triangles at in current.indices[i][1]
                //This allows us to traverse leaf Nodes with multiple entries
                for (int i = 0; i < current.indices.size(); i++) {
                    const auto& mesh = m_pScene->meshes[current.indices[i][0]];
                    const auto& tri = mesh.triangles[current.indices[i][1]];
                    if (tri.length > 0) {
                        const Vertex& v0 = mesh.vertices[tri[0]];
                        const Vertex& v1 = mesh.vertices[tri[1]];
                        const Vertex& v2 = mesh.vertices[tri[2]];
                        //changed from .position
                        if (intersectRayWithTriangle(v0, v1, v2, ray, hitInfo)) {
                            hitInfo.material = mesh.material;
                            hit = true;
                            //Visual Debug 3.5 Traversal Basic
                            //Here we draw all intersected Triangles
                            if (visualDebug.TraversalBasic) {
                                drawTriangle(mesh, current.indices[i][1]);
                            }
                        }
                    }
                }
            }

            //If not a leaf node, access both left and right child
            else {
                bool leftHit = false;
                bool rightHit = false;

                //Recursive call left child
                if (current.indices[0].size() > 0) {
                    leftHit = this->intersectCustomBasic(ray, hitInfo, depth + 1, current.indices[0][0], visualDebug, recursiveColor);
                }
                //recursive call right child
                if (current.indices[1].size() > 0) {
                    rightHit = this->intersectCustomBasic(ray, hitInfo, depth + 1, current.indices[1][0], visualDebug, recursiveColor);
                }

                //hit needs to be set to true if one of the children had an intersection
                if (leftHit) {
                    hit = true;
                }
                if (rightHit) {
                    hit = true;
                }
            }

        }
    }
    //Case for spheres (was here before)
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    
    //Extra thing I coded. This will draw all not visited nodes (by looking at all the visited ones)
    if (visualDebug.notVisitedNodesTraversal && depth == 0) {
        for (int i = 0; i < nodes.size(); i++) {
            if (std::find(hitNodes.begin(), hitNodes.end(), i) != hitNodes.end()) {
                continue;
            }
            else {
                drawAABB(nodes[i].box, DrawMode::Wireframe, glm::vec3(1.0f, 0.0f, 0.0f), 1);
            }
        }
    }
    if (depth == 0) {
        hitNodes.clear();
    }

    //Return hit
    return hit;
}

//Advanced Traversal
bool BoundingVolumeHierarchy::intersectCustomAdvanced(Ray& ray, HitInfo& hitInfo, float depth, int nodeIndex, const VisualDebug& visualDebug) const {
    bool hit = false;
    //recursiveColor is used for the semi-random color that is applied to each AABB at each recursive iteration
    //Though, the advanced traversal uses iteration instead of recursion, for ease of use (so I have similar variables in basic and advanced)
    glm::vec3 recursiveColor = glm::vec3{0.3f, 0.9f, 0.1f};
    //Create new priority queue
    std::priority_queue<std::tuple<int, float>, std::vector<std::tuple<int, float>>, CompareDistance> pq;

    //Push all Leafnodes whose AABBs intersect with ray and into PQ
    //PQ is min-first and is sorted based on the distance from the ray origin to the hit AABB
    for (int i = 0; i < nodes.size(); i++) {
        Node currentNode = nodes[i];
        float old_t = ray.t;
        //Extra thing I did, this draws all the non-visited nodes
        if (!currentNode.isLeaf) {
            if (visualDebug.notVisitedNodesTraversal) {
                drawAABB(currentNode.box, DrawMode::Wireframe, glm::vec3(1.0f, 0.0f, 0.0f), 1);
            }
            continue;
        }
        bool rayAABB = intersectRayWithShape(currentNode.box, ray);
        float distance = ray.t;
        ray.t = old_t;
        std::tuple<int, float> currentTuple = std::make_tuple(i, distance);
        if (rayAABB) {
            pq.push(currentTuple);
        }
    }

    //Create reference tuple (this value is not achievable  naturally) 
    std::tuple<int, float> currentPopped = std::make_tuple(-1, -1.0f);

    //Get first PQ element
    if (!pq.empty()) {
        currentPopped = pq.top();
        pq.pop();
    }

    //In case there is no leaf node index, we are left with <-1,-1> and never enter the for loop
    //Otherwise enter for loop with new leaf node index
    if (get<0>(currentPopped) != -1 && get<1>(currentPopped) != -1.0f) {
        //Get the actual Leaf Node (before we worked with tuples)
        Node currentLeaf = nodes[get<0>(currentPopped)];
            //Visual Debug 3.5 Traversal Advanced
            //This draws the first AABB that is visited
            if (visualDebug.TraversalAdvanced) {
                drawAABB(currentLeaf.box, DrawMode::Wireframe, glm::vec3(0.12f, 0.23f, 0.40f), 1);
            }
            //Get meshes and draw triangle(s)
            for (int i = 0; i < currentLeaf.indices.size(); i++) {
                const auto& mesh = m_pScene->meshes[currentLeaf.indices[i][0]];
                const auto& tri = mesh.triangles[currentLeaf.indices[i][1]];
                if (tri.length > 0) {
                    const Vertex v0 = mesh.vertices[tri[0]];
                    const Vertex v1 = mesh.vertices[tri[1]];
                    const Vertex v2 = mesh.vertices[tri[2]];
                    //changed from .position
                    if (intersectRayWithTriangle(v0, v1, v2, ray, hitInfo)) {
                        hitInfo.material = mesh.material;
                        hit = true;
                        //Extra thing I did, this draws all the non-visited nodes
                        if (visualDebug.notVisitedNodesTraversal && !pq.empty()) {
                            pq.pop();
                            if (!pq.empty()) {
                                currentPopped = pq.top();
                            }
                            while (!pq.empty()) {
                                drawAABB(nodes[get<0>(currentPopped)].box, DrawMode::Wireframe, glm::vec3(1.0f, 0.0f, 0.0f), 1);
                                currentPopped = pq.top();
                                pq.pop();
                            }
                        }
                        //Visual Debug 3.5 Traversal Advanced
                        //Draw the intersected triangle(s)
                        if (visualDebug.TraversalAdvanced) {
                            drawTriangle(mesh, currentLeaf.indices[i][1]);
                        }
                    }
                }
            }
            //If we found a hit, we can stop at this point
            if (hit) {
                return hit;
            }
            //If the first leaf node did not have a hit, and the PQ is not empty, we might have more leaf nodes in the PQ
            while (!hit && !pq.empty()) {
                currentPopped = pq.top();
                pq.pop();

                //Visual Debug 3.5 Advanced
                //This draws any following AABB in a semi-random color
                if (visualDebug.TraversalBasic) {
                    drawAABB(nodes[get<0>(currentPopped)].box, DrawMode::Wireframe, recursiveColor, 1);
                    float r = fmod(recursiveColor.x + 0.05f + recursiveColor.z * 2 / 3, 1.0f);
                    float g = fmod(recursiveColor.y + 0.05f + recursiveColor.y * 1 / 4, 1.0f);
                    float b = fmod(recursiveColor.z + 0.05f + recursiveColor.x * 1 / 2, 1.0f);
                    while ((r - 1.0f < 0.15f) && (g - 0.0f < 0.15f) && (b - 0.0f < 0.15f)) {
                        float r = fmod(recursiveColor.x + 0.05f + recursiveColor.z * 2 / 3, 1.0f);
                        float g = fmod(recursiveColor.y + 0.05f + recursiveColor.y * 1 / 4, 1.0f);
                        float b = fmod(recursiveColor.z + 0.05f + recursiveColor.x * 1 / 2, 1.0f);
                    }
                    recursiveColor = glm::vec3{ r,g,b };
                }
                //Call the iterative part of the code until we loop over all remaining nodes
                //The method is called "..recursive" for legacy reasons
                hit = intersectCustomRecursive(ray, hitInfo, currentPopped, visualDebug, recursiveColor);
                if (hit) {
                
                    //Extra thing I did, this draws all the non-visited nodes
                    if (visualDebug.notVisitedNodesTraversal && !pq.empty()) {
                        pq.pop();
                        if (!pq.empty()) {
                            currentPopped = pq.top();
                        }
                        while (!pq.empty()) {
                            drawAABB(nodes[get<0>(currentPopped)].box, DrawMode::Wireframe, glm::vec3(1.0f, 0.0f, 0.0f), 1);
                            currentPopped = pq.top();
                            pq.pop();
                        }
                    }

                    //If we found a hit, no further searching is needed
                    return true;
                }
            }
    }
    //Sphere part
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    //Return hit
    return hit;

}

//Recursive part for the advanced traversal
bool BoundingVolumeHierarchy::intersectCustomRecursive(Ray& ray, HitInfo& hitInfo, std::tuple<int, float>  currentPopped, const VisualDebug& visualDebug, glm::vec3 recursiveColor) const
{
    bool hit = false;
    //Get the node
    Node currentLeaf = nodes[get<0>(currentPopped)];
    //Check that it is a leaf node
    if (currentLeaf.isLeaf) {
        //Visual debug 3.5 Traversal advanced
        if (visualDebug.TraversalAdvanced) {
            drawAABB(currentLeaf.box, DrawMode::Wireframe, recursiveColor, 1);
        }
        //Calculate if there is a hit
        for (int i = 0; i < currentLeaf.indices.size(); i++) {
            const auto& mesh = m_pScene->meshes[currentLeaf.indices[i][0]];
            const auto& tri = mesh.triangles[currentLeaf.indices[i][1]];
            if (tri.length > 0) {
                const Vertex v0 = mesh.vertices[tri[0]];
                const Vertex v1 = mesh.vertices[tri[1]];
                const Vertex v2 = mesh.vertices[tri[2]];
                //changed from .position
                if (intersectRayWithTriangle(v0, v1, v2, ray, hitInfo)) {
                    hitInfo.material = mesh.material;
                    hit = true;
                    //Visual Debug 3.5 Traversal Advanced
                    if (visualDebug.TraversalAdvanced) {
                        drawTriangle(mesh, currentLeaf.indices[i][1]);
                    }
                    //If we found a hit, no further searching is needed
                    //return hit;
                }
            }
        }
    }
    // Intersect with spheres.
    //for (const auto& sphere : m_pScene->spheres)
    //    hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}

//void BoundingVolumeHierarchy::loadVertexNormals() const {
//    for (auto& mesh : m_pScene->meshes) {
//        for (size_t i = 0; i < mesh.vertices.size(); i++) {
//            glm::vec3 vertexNormals(0.0f);
//            Vertex& v = mesh.vertices[i];
//            for (const auto& tri : mesh.triangles) {
//                const glm::vec3 v0 = mesh.vertices[tri[0]].position;
//                const glm::vec3 v1 = mesh.vertices[tri[1]].position;
//                const glm::vec3 v2 = mesh.vertices[tri[2]].position;
//                if (v0 == v.position || v1 == v.position || v2 == v.position)
//                {
//                    vertexNormals += glm::normalize(glm::cross(v1 - v0, v2 - v0));
//                }
//            };
//            v.normal = glm::normalize(vertexNormals);
//        }
//    }
//}

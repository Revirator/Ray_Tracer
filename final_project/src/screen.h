#pragma once
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <filesystem>
#include <vector>

class Screen {
public:
    Screen(const glm::ivec2& resolution);

    void clear(const glm::vec3& color);
    void setPixel(int x, int y, const glm::vec3& color);
    void setBloom(int x, int y, const glm::vec3& color);
    glm::vec3 getPixel(int x, int y);
    glm::vec3 getBloom(int x, int y);


    void writeBitmapToFile(const std::filesystem::path& filePath);
    void draw();

public:
    glm::ivec2 m_resolution;
    std::vector<glm::vec3> m_textureData;
    std::vector<glm::vec3> bloomData;
private:
    uint32_t m_texture;
};

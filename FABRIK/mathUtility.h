#pragma once
#include <glm/glm.hpp>

glm::mat3x3 crossMatrix(glm::vec3 a);

//ToDo: Transpose results?
glm::mat2x2 vectorToVectorRotation(glm::vec2 a, glm::vec2 b);
glm::mat3x3 vectorToVectorRotation(glm::vec3 a, glm::vec3 b);

glm::vec2 calc2dEllipsePoint(float width, float height, float currentAngle);

float angleBetweenVecs(glm::vec3 a, glm::vec3 b);
#pragma once
#include <glm/glm.hpp>

glm::mat3x3 crossMatrix(glm::vec3 a);
glm::mat3x3 vectorToVectorRotation(glm::vec3 a, glm::vec3 b);

float angleBetweenVecs(glm::vec3 a, glm::vec3 b);
#include "mathUtility.h"
/*
*	Calculates the cross-product matrix for a given vector a
*/
glm::mat3x3 crossMatrix(glm::vec3 a) {
	return glm::mat3x3(0, -a.z, a.y, a.z, 0, -a.x, -a.y, a.x, 0);
}

/*
*	Find a rotation matrix R that rotates unit vector a onto unit vector b (does not work for <(a,b) = -1)
*	Source: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
*/
/*	Example code:
glm::vec3 a = glm::vec3(0.5f, -2.0f, 3.0f);
glm::vec3 b = glm::vec3(-0.3f, 1.0f, 0.0f);
a = glm::normalize(a);
b = glm::normalize(b);

b = vectorToVectorRotation(a, b)*b;
std::cout << "Vector a x: " << a.x << ", y:" << a.y << ", z:" << a.z << std::endl;
std::cout << "Vector b x: " << b.x << ", y:" << b.y << ", z:" << b.z << std::endl;
*/
glm::mat3x3 vectorToVectorRotation(glm::vec3 a, glm::vec3 b) {
	a = glm::normalize(a);
	b = glm::normalize(b);

	glm::vec3 v = glm::cross(a, b);
	float s = glm::length(v);
	float c = glm::dot(a, b);

	glm::mat3x3 crossV = crossMatrix(v);

	return glm::mat3x3(1.0f) + crossV + crossV*crossV*(1.0f / (1.0f + c));
}

float angleBetweenVecs(glm::vec3 a, glm::vec3 b)
{
	return glm::acos((glm::dot(a, b)) / (glm::length(a)*glm::length(b)));
}

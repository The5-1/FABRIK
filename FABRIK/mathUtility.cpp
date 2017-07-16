#include "mathUtility.h"
#include <iostream>
#define M_PI 3.14159265359

/*
*	Calculates the cross-product matrix for a given vector a
*/
glm::mat3x3 crossMatrix(glm::vec3 a) {
	return glm::mat3x3(0, -a.z, a.y, a.z, 0, -a.x, -a.y, a.x, 0);
}

glm::mat2x2 vectorToVectorRotation(glm::vec2 a, glm::vec2 b)
{
	a = glm::normalize(a);
	b = glm::normalize(b);

	float x1 = a.x;
	float y1 = a.y;
	float x2 = b.x;
	float y2 = b.y;

	return glm::mat2x2(x1*x2+y1*y2, -(x1*y2-x2*y1), x1*y2-x2*y1, x1*x2+y1*y2);
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

	if (glm::cross(a, b) == glm::vec3(0.0f)) {
		return glm::mat3x3(1.0f);
	}

	if (glm::dot(a, b) / (glm::length(a)*glm::length(b)) == -1) {
		return glm::mat3x3(-1.0f);
	}

	glm::vec3 v = glm::cross(a, b);
	float s = glm::length(v);
	float c = glm::dot(a, b);

	glm::mat3x3 crossV = crossMatrix(v);

	return glm::mat3x3(1.0f) + crossV + crossV*crossV*(1.0f / (1.0f + c));
}

glm::vec2 calc2dEllipsePoint(float width, float height, float currentAngle)
{
	//Web: https://math.stackexchange.com/questions/22064/calculating-a-point-that-lies-on-an-ellipse-given-an-angle
	float x = (width * height) / (glm::sqrt(height*height + width*width*glm::tan(currentAngle)*glm::tan(currentAngle)));
	float y = (width * height * glm::tan(currentAngle)) / (glm::sqrt(height*height + width*width*glm::tan(currentAngle)*glm::tan(currentAngle)));

	//Clamp the angle to [0, 2*Pi]
	currentAngle = glm::mod(currentAngle, 2.0f*(float)M_PI);

	if ((currentAngle >= 0 && currentAngle <= M_PI / 2.0f) || (currentAngle >= 1.5f*M_PI && currentAngle <= 2.0f*M_PI)) {
		//std::cout << "debug me 1" << std::endl;
		return glm::vec2(x, y);
	}
	else {
		//std::cout << "debug me 2" << std::endl;
		return glm::vec2(-x, -y);
	}
}

float angleBetweenVecs(glm::vec3 a, glm::vec3 b)
{
	return glm::acos((glm::dot(a, b)) / (glm::length(a)*glm::length(b)));
}

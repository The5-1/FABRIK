#include "Constraint.h"
#include "Segment.h"
#include "mathUtility.h"
#include <glm\glm.hpp>
#include <iostream>


#define M_PI 3.14159265359

Constraint::Constraint()
{
}

Constraint::Constraint(glm::vec3 _color, bool _render, bool _active)
{
	this->color = _color;
	this->render = _render;
	this->active = _active;
}

Constraint::~Constraint()
{
}

ConeConstraint::ConeConstraint(glm::vec3 _color, bool _render, bool _active)
{
	this->color = _color;
	this->render = _render;
	this->active = _active;
}

ConeConstraint::ConeConstraint(float _coneAngle) {
	this->coneAngle = _coneAngle;
}

float ConeConstraint::getConeAngle()
{
	return this->coneAngle;
}

void ConeConstraint::setConeAngle(float _coneAngle)
{
	this->coneAngle = _coneAngle;
}

glm::vec3 ConeConstraint::calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent)
{
	float coneAngle = this->coneAngle;

	glm::vec3 coneAxis = (parent.endJoint - parent.startJoint);
	glm::vec3 normalConeAxis = glm::normalize(coneAxis);

	glm::vec3 unconstraintedVec = current.endJoint - current.startJoint;

	glm::vec3 projectionVec = glm::dot(unconstraintedVec, normalConeAxis) * normalConeAxis;
	float projectionLength = glm::length(projectionVec);

	if (glm::dot(coneAxis, projectionVec) < 0) {
		projectionVec = -projectionVec;
	}

	glm::vec3 projectionPoint = current.startJoint + projectionVec;

	float constraintRadius = projectionLength * glm::tan(coneAngle);
	float projectionRadius = glm::length(current.endJoint - projectionPoint);

	if (constraintRadius < projectionRadius) {
		glm::vec3 projectionOnCone = projectionPoint + constraintRadius * glm::normalize(current.endJoint - projectionPoint);
		
		return current.startJoint + current.length * glm::normalize(projectionOnCone - current.startJoint);
	}

	return current.endJoint;
}

/*
* Cone Constraint via paper method:
* Transform problem to a 2D-ellipse, inside-outside-test, backtransformation
*/
//glm::vec3 ConeConstraint::calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent)
//{
//	std::cout << "Solve with cone" << std::endl;
//	//ToDo: Cones in segments speichern
//	float coneAngle = M_PI / 6;
//
//	glm::vec3 d = (parent.endJoint - parent.startJoint);
//	//Rotation
//	glm::mat3x3 rotation = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), d);
//	d = glm::normalize(rotation * d);
//	std::cout << "d ( " << d.x << ", " << d.y << ", " << d.z << ") mit Laenge " << glm::length(d) << std::endl;
//
//	//Projection
//	glm::vec3 c = rotation * (current.endJoint - current.startJoint);
//	std::cout << "c ( " << c.x << ", " << c.y << ", " << c.z << ") mit Laenge " << glm::length(c) << std::endl;
//	c.y = glm::abs(c.y);
//	float height = c.y;
//	float radius = height * glm::tan(coneAngle);
//
//	std::cout << "Radius: " << radius << std::endl;
//
//	glm::vec3 p = glm::dot(c, d) * d;
//	if (glm::dot(c, d) < 0) {
//		p = -p;
//	}
//
//	std::cout << "p ( " << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
//
//	glm::vec3 z = c - p;
//	std::cout << "z ( " << z.x << ", " << z.y << ", " << z.z << ")" << std::endl;
//
//
//	if ((z.x*z.x) / (radius*radius) + (z.z*z.z) / (radius*radius) > 1) {
//		std::cout << " Outside of ellipse " << std::endl;
//		//float sigma = atan2(current.endJoint.z, current.endJoint.x);
//		float sigma = atan2(z.z, z.x);
//
//		p.x = radius * glm::cos(sigma);
//		//p.y = height;
//		p.z = radius * glm::sin(sigma);
//
//		p = transpose(rotation) * p;
//		std::cout << "p ( " << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
//		return current.startJoint + current.length * glm::normalize(p);
//	}
//
//	return unconstrainted;
//}

PlaneConstraint::PlaneConstraint(glm::vec3 _planeNormal)
{
	this->planeNormal = glm::normalize(_planeNormal);
}

glm::vec3 PlaneConstraint::calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent)
{
	//Web: https://www.maplesoft.com/support/help/Maple/view.aspx?path=MathApps/ProjectionOfVectorOntoPlane

	glm::vec3 currentVector = current.endJoint - current.startJoint;
	glm::vec3 projectionOnPlane = current.length * glm::normalize(currentVector - glm::dot(currentVector, planeNormal)*planeNormal);
	glm::vec3 projectionVec = projectionOnPlane + current.startJoint;

	return projectionVec;
}

HingeConstraint::HingeConstraint(float _radius)
{
	this->radius = _radius;
}

glm::vec3 HingeConstraint::calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent)
{
	glm::vec3 currentVector = current.endJoint - current.startJoint;
	glm::vec3 planeNormal = parent.endJoint  - parent.startJoint;

	glm::vec3 projectionOnPlane = currentVector - glm::dot(currentVector, planeNormal)*planeNormal;
	glm::vec3 projectionPoint = projectionOnPlane + current.startJoint;
	float length = glm::length(current.startJoint - projectionPoint);

	if (length > 1) {
		projectionOnPlane = this->radius * glm::normalize(projectionOnPlane);
	}

	return glm::vec3();
}

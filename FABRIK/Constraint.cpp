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

void ConeConstraint::calcConstraintedPointReference(glm::vec3 unconstrainted, Segment & current, Segment & parent)
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

		current.endJoint = current.startJoint + current.length * glm::normalize(projectionOnCone - current.startJoint);
	}
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

void PlaneConstraint::setPlaneNormal(glm:: vec3 _planeNormal)
{
	this->planeNormal = _planeNormal;
}

glm::vec3 PlaneConstraint::calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent)
{
	//Web: https://www.maplesoft.com/support/help/Maple/view.aspx?path=MathApps/ProjectionOfVectorOntoPlane

	glm::vec3 currentVector = current.endJoint - current.startJoint;
	glm::vec3 projectionOnPlane = current.length * glm::normalize(currentVector - glm::dot(currentVector, this->planeNormal)*this->planeNormal);
	glm::vec3 projectionVec = projectionOnPlane + current.startJoint;

	return projectionVec;
}

void PlaneConstraint::calcConstraintedPointReference(glm::vec3 unconstrainted, Segment & current, Segment & parent)
{
	glm::vec3 currentVector = current.endJoint - current.startJoint;
	glm::vec3 projectionOnPlane = current.length * glm::normalize(currentVector - glm::dot(currentVector, this->planeNormal)*this->planeNormal);
	glm::vec3 projectionVec = projectionOnPlane + current.startJoint;

	current.endJoint = projectionVec;
}

HingeConstraint::HingeConstraint(float _radius)
{
	this->radius = _radius;
}

void HingeConstraint::setHingeRadius(float _radius)
{
	this->radius = _radius;
}

glm::vec3 HingeConstraint::calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent)
{
	//This does not work for the normal FABRIK because current.startJoint has to be modified not current.endJoint
	//Solution: change from return type to modify references
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

void HingeConstraint::calcConstraintedPointReference(glm::vec3 unconstrainted, Segment & current, Segment & parent)
{	
	float maxRadius = this->radius;

	glm::vec3 currentVector = glm::normalize(current.endJoint - current.startJoint);
	glm::vec3 planeNormal = glm::normalize(parent.endJoint - parent.startJoint);

	glm::vec3 projectionOnPlane = currentVector - glm::dot(currentVector, planeNormal)*planeNormal;
	glm::vec3 projectionPoint = projectionOnPlane + current.startJoint;

	float length = glm::length(projectionOnPlane);

	if (length > maxRadius) {
		projectionOnPlane = maxRadius * glm::normalize(projectionOnPlane);
	}

	current.startJoint = current.startJoint + projectionOnPlane;
}

EllipticConeConstraint::EllipticConeConstraint(float _width0, float _width1, float _width2, float _width3)
{
	this->width0 = _width0;
	this->width1 = _width1;
	this->width2 = _width2;
	this->width3 = _width3;
}

glm::vec3 EllipticConeConstraint::calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent)
{
	glm::vec3 translation = parent.startJoint;

	glm::vec3 _start = glm::vec3(0.0f); //Translate vectorstart to (0,  0, 0)

	glm::vec3 _e1 = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 _a = parent.endJoint - translation;
	glm::vec3 _b = current.endJoint - translation;

	glm::mat3x3 _rot = (vectorToVectorRotation(_e1, _a));

	glm::vec3 _rotStart = glm::length(_start) * glm::normalize(_rot * _start);

	glm::vec3 _rotA = glm::length(_a) * glm::normalize(_rot * (_a));

	//Point or vector? (Point seems to be correct, why?)
	glm::vec3 _rotB = glm::length(_b) * glm::normalize(_rot * (_b));
	//glm::vec3 _rotB = glm::length(_b - _a) * glm::normalize(_rot * (_b - _a));

	/*
	//This is the elliptic math
	*/
	glm::vec3 _coneAxis = (_a - _start);
	glm::vec3 _normalConeAxis = glm::normalize(_coneAxis); //this has to be normalized for the projection

	glm::mat3x3 _rotation = _rot;

	glm::vec3 _unconstraintedVec;// = _b - _a;

	_normalConeAxis = glm::normalize(_rotation * _normalConeAxis);  //this should be glm::vec3(0.0f, 1.0f, 0.0f)

	_unconstraintedVec = glm::length(_b) * glm::normalize(_rotation * _b);

	//ProjectionVec drawn in yellow
	glm::vec3 projectionVec = glm::dot(_unconstraintedVec, _normalConeAxis) * _normalConeAxis;

	float projectionLength = glm::length(_rotB - projectionVec);

	if (glm::dot(_coneAxis, projectionVec) < 0) {
		projectionVec = -projectionVec;
	}

	//Find in which cone-segment we are
	float currentWidth, currentHeight;
	if (_rotB.x >= 0 && _rotB.z >= 0) {
		currentWidth = this->width0;
		currentHeight = this->width1;
	}
	else if (_rotB.x <= 0 && _rotB.z >= 0) {
		currentWidth = this->width2;
		currentHeight = this->width1;
	}
	else if (_rotB.x <= 0 && _rotB.z <= 0) {
		currentWidth = this->width2;
		currentHeight = this->width3;
	}
	else if (_rotB.x >= 0 && _rotB.z <= 0) {
		currentWidth = this->width0;
		currentHeight = this->width3;
	}

	glm::vec3 constraintProjection;
	glm::vec3 _ellipticB;
	glm::vec3 _nonProjectedResult;

	if ((_rotB.x*_rotB.x) / (currentWidth*currentWidth) + (_rotB.z*_rotB.z) / (currentHeight*currentHeight) > 1) {
		float sigma = atan2(_rotB.z, _rotB.x);
		constraintProjection = glm::vec3(currentWidth * glm::cos(sigma), _rotA.y + 1.0f, currentHeight * glm::sin(sigma));
		_ellipticB = glm::length(_rotB - _rotA) * (constraintProjection - _rotA) + _rotA;

		_nonProjectedResult = glm::transpose(_rot) * _ellipticB + translation;
		current.endJoint = glm::length(_a - _b) * glm::normalize(_nonProjectedResult - current.startJoint) + current.startJoint;

	}
	else {
		return current.endJoint;
	}
}

void EllipticConeConstraint::setAxis(glm::vec3 _xAxis, glm::vec3 _yAxis, glm::vec3 _zAxis)
{
	this->xAxis = _xAxis;
	this->yAxis = _yAxis;
	this->zAxis = _zAxis;
}

//void EllipticConeConstraint::calcConstraintedPointReference(glm::vec3 unconstrainted, Segment & current, Segment & parent)
//{
//	//All constraint cones have the default height 1.0f
//	float height = 1.0f;
//
//	glm::vec3 e0 = glm::vec3(1.0f, 0.0f, 0.0f);
//	glm::vec3 e1 = glm::vec3(0.0f, 1.0f, 0.0f);
//	glm::vec3 e2 = glm::vec3(0.0f, 0.0f, 1.0f);
//
//	glm::vec3 coneAxis = (parent.endJoint - parent.startJoint);
//	glm::vec3 normalConeAxis = glm::normalize(coneAxis);
//
//	glm::mat3x3 rotation = vectorToVectorRotation(e1, coneAxis);
//
//	glm::vec3 unconstraintedVec = current.endJoint - current.startJoint;
//
//	normalConeAxis = normalize(rotation * normalConeAxis);
//	unconstraintedVec = rotation * unconstraintedVec;
//
//
//	glm::vec3 projectionVec = glm::dot(unconstraintedVec, normalConeAxis) * normalConeAxis;
//	float projectionLength = glm::length(projectionVec);
//
//	if (glm::dot(coneAxis, projectionVec) < 0) {
//		projectionVec = -projectionVec;
//	}
//
//	//Find in which cone-segment we are
//	float currentWidth, currentHeight;
//	if (projectionVec.x >= 0 && projectionVec.z >= 0) {
//		currentWidth = this->width0;
//		currentHeight = this->width1;
//	}
//	else if (projectionVec.x <= 0 && projectionVec.z >= 0) {
//		currentWidth = this->width2;
//		currentHeight = this->width1;
//	}
//	else if (projectionVec.x <= 0 && projectionVec.z <= 0) {
//		currentWidth = this->width2;
//		currentHeight = this->width3;
//	}
//	else if (projectionVec.x >= 0 && projectionVec.z <= 0) {
//		currentWidth = this->width0;
//		currentHeight = this->width3;
//	}
//
//	//std::cout << projectionVec.x << " " << projectionVec.y << " " << projectionVec.z << std::endl;
//
//	if (projectionVec.y != 1.0f) {
//		projectionVec = (1.0f / projectionVec.y) * projectionVec;
//	}
//	//std::cout <<"Projection:" << projectionVec.x << " " << projectionVec.y << " " << projectionVec.z << std::endl;
//
//	//if ((projectionVec.x*projectionVec.x) / (currentWidth*currentWidth) + (projectionVec.z*projectionVec.z) / (currentHeight*currentHeight) > 1) {
//	float sigma = atan2(projectionVec.z, projectionVec.x);
//	//glm::vec3 constraintProjection = glm::vec3(currentWidth*glm::cos(sigma), 1.0f, currentHeight*glm::sin(sigma));
//	glm::vec3 constraintProjection = glm::vec3(currentWidth*glm::sin(sigma), 1.0f, currentHeight*glm::cos(sigma));
//
//	current.endJoint = current.startJoint + current.length*glm::normalize(glm::transpose(rotation)*constraintProjection);
//}

//void EllipticConeConstraint::calcConstraintedPointReference(glm::vec3 unconstrainted, Segment & current, Segment & parent)
//{
//
//	std::cout << "calcConstraintedPointReference" << std::endl;
//
//	//All constraint cones have the default height 1.0f
//	float height = 1.0f; 
//
//	glm::vec3 coneAxis = (parent.endJoint - parent.startJoint);
//	glm::vec3 normalConeAxis = glm::normalize(coneAxis);
//
//	glm::mat3x3 rotation = glm::transpose(vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), coneAxis));
//
//	glm::vec3 unconstraintedVec = current.endJoint - current.startJoint; 
//
//	normalConeAxis = normalize(rotation * normalConeAxis);
//	unconstraintedVec = rotation * unconstraintedVec;
//
//
//	glm::vec3 projectionVec = glm::dot(unconstraintedVec, normalConeAxis) * normalConeAxis;
//	float projectionLength = glm::length(projectionVec);
//
//	if (glm::dot(coneAxis, projectionVec) < 0) {
//		projectionVec = -projectionVec;
//	}
//
//	//Find in which cone-segment we are
//	float currentWidth, currentHeight;
//	if (projectionVec.x >= 0 && projectionVec.z >= 0) {
//		currentWidth = this->width0;
//		currentHeight = this->width1;
//	}
//	else if (projectionVec.x <= 0 && projectionVec.z >= 0) {
//		currentWidth = this->width2;
//		currentHeight = this->width1;
//	}
//	else if (projectionVec.x <= 0 && projectionVec.z <= 0) {
//		currentWidth = this->width2;
//		currentHeight = this->width3;
//	}
//	else if (projectionVec.x >= 0 && projectionVec.z <= 0) {
//		currentWidth = this->width0;
//		currentHeight = this->width3;
//	}
//
//	//std::cout << projectionVec.x << " " << projectionVec.y << " " << projectionVec.z << std::endl;
//	
//	if (projectionVec.y != 1.0f) {
//		projectionVec = (1.0f / projectionVec.y) * projectionVec;
//	}
//	//std::cout <<"Projection:" << projectionVec.x << " " << projectionVec.y << " " << projectionVec.z << std::endl;
//
//	if ((projectionVec.x*projectionVec.x) / (currentWidth*currentWidth) + (projectionVec.z*projectionVec.z) / (currentHeight*currentHeight) > 1) {
//		float sigma = atan2(projectionVec.z, projectionVec.x);
//		//glm::vec3 constraintProjection = glm::vec3(currentWidth*glm::cos(sigma), 1.0f, currentHeight*glm::sin(sigma));
//		glm::vec3 constraintProjection = glm::vec3(currentWidth*glm::sin(sigma), 1.0f, currentHeight*glm::cos(sigma));
//
//		//current.endJoint = current.startJoint + current.length*glm::normalize(glm::inverse(rotation)*constraintProjection);
//		current.endJoint = current.startJoint + current.length*glm::normalize(glm::transpose(rotation)*constraintProjection);
//
//		//std::cout << "End Joint:" << current.endJoint.x << " " << current.endJoint.y << " " << current.endJoint.z << std::endl;
//	}
//
//	/*
//	glm::vec3 projectionPoint = current.startJoint + projectionVec;
//
//	float constraintRadius = projectionLength * glm::tan(coneAngle);
//	float projectionRadius = glm::length(current.endJoint - projectionPoint);
//
//	if (constraintRadius < projectionRadius) {
//		glm::vec3 projectionOnCone = projectionPoint + constraintRadius * glm::normalize(current.endJoint - projectionPoint);
//
//		current.endJoint = current.startJoint + current.length * glm::normalize(projectionOnCone - current.startJoint);
//	}*/
//}

void EllipticConeConstraint::calcConstraintedPointReference(glm::vec3 unconstrainted, Segment & current, Segment & parent)
{
	glm::vec3 translation = parent.startJoint;

	glm::vec3 _start = glm::vec3(0.0f); //Translate vectorstart to (0,  0, 0)

	glm::vec3 _e1 = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 _a = parent.endJoint - translation;
	glm::vec3 _b = current.endJoint - translation;

	glm::mat3x3 _rot = (vectorToVectorRotation(_e1, _a));

	glm::vec3 _rotStart = glm::length(_start) * glm::normalize(_rot * _start);

	glm::vec3 _rotA = glm::length(_a) * glm::normalize(_rot * (_a));

	//Point or vector? (Point seems to be correct, why?)
	glm::vec3 _rotB = glm::length(_b) * glm::normalize(_rot * (_b));
	//glm::vec3 _rotB = glm::length(_b - _a) * glm::normalize(_rot * (_b - _a));

	/*
	//This is the elliptic math
	*/
	glm::vec3 _coneAxis = (_a - _start);
	glm::vec3 _normalConeAxis = glm::normalize(_coneAxis); //this has to be normalized for the projection

	glm::mat3x3 _rotation = _rot;

	glm::vec3 _unconstraintedVec;// = _b - _a;

	_normalConeAxis = glm::normalize(_rotation * _normalConeAxis);  //this should be glm::vec3(0.0f, 1.0f, 0.0f)

	_unconstraintedVec = glm::length(_b) * glm::normalize(_rotation * _b);

	//ProjectionVec drawn in yellow
	glm::vec3 projectionVec = glm::dot(_unconstraintedVec, _normalConeAxis) * _normalConeAxis;

	float projectionLength = glm::length(_rotB - projectionVec);

	if (glm::dot(_coneAxis, projectionVec) < 0) {
		projectionVec = -projectionVec;
	}

	//Find in which cone-segment we are
	float currentWidth, currentHeight;
	if (_rotB.x >= 0 && _rotB.z >= 0) {
		currentWidth = this->width0;
		currentHeight = this->width1;
	}
	else if (_rotB.x <= 0 && _rotB.z >= 0) {
		currentWidth = this->width2;
		currentHeight = this->width1;
	}
	else if (_rotB.x <= 0 && _rotB.z <= 0) {
		currentWidth = this->width2;
		currentHeight = this->width3;
	}
	else if (_rotB.x >= 0 && _rotB.z <= 0) {
		currentWidth = this->width0;
		currentHeight = this->width3;
	}

	glm::vec3 constraintProjection;
	glm::vec3 _ellipticB;
	glm::vec3 _nonProjectedResult;

	if ((_rotB.x*_rotB.x) / (currentWidth*currentWidth) + (_rotB.z*_rotB.z) / (currentHeight*currentHeight) > 1) {
		float sigma = atan2(_rotB.z, _rotB.x);
		constraintProjection = glm::vec3(currentWidth * glm::cos(sigma), _rotA.y + 1.0f, currentHeight * glm::sin(sigma));
		_ellipticB = glm::length(_rotB - _rotA) * (constraintProjection - _rotA) + _rotA;

		_nonProjectedResult = glm::transpose(_rot) * _ellipticB + translation;
		current.endJoint = glm::length(_a - _b) * glm::normalize(_nonProjectedResult - current.startJoint) + current.startJoint;

	}
	else {
		return;
	}	
}


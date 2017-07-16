#pragma once
#include <glm\glm.hpp>

class Segment;

class Constraint
{
//Protected instead of private so children can use them
protected:
	glm::vec3 color;
	bool render = true;
	bool active = true;
public:
	Constraint();
	Constraint(glm::vec3 _color, bool _render, bool _active);
	~Constraint();

	virtual glm::vec3 calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent) = 0;
	virtual void calcConstraintedPointReference(glm::vec3 unconstrainted, Segment& current, Segment& parent) = 0;
};

class ConeConstraint : public Constraint {
private:
	float angle;
	float radius;
	int program;
	float coneAngle;
	glm::vec3 coneAxis;

public:
	ConeConstraint(glm::vec3 _color, bool _render, bool _active);
	ConeConstraint(float _angle, float _radius, const int program);
	ConeConstraint(float _coneAngle);

	float getConeAngle();
	void setConeAngle(float _coneAngle);

	glm::vec3 calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent);
	void calcConstraintedPointReference(glm::vec3 unconstrainted, Segment& current, Segment& parent);
};

class EllipticConeConstraint : public Constraint {
private:
	float width0, width1, width2, width3;
	glm::vec3 xAxis = glm::vec3(1.0f, 0.0f, 0.0f);
	glm::vec3 yAxis = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 zAxis = glm::vec3(0.0f, 0.0f, 1.0f);

public:
	EllipticConeConstraint(float _width0, float _width1, float _width2, float _width3);

	//float getConeAngle();
	//void setConeAngle(float _coneAngle);

	glm::vec3 calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent);
	void setAxis(glm::vec3 _xAxis, glm::vec3 _yAxis, glm::vec3 _zAxis);
	void calcConstraintedPointReference(glm::vec3 unconstrainted, Segment& current, Segment& parent);
};

class PlaneConstraint : public Constraint {
private:
	glm::vec3 planeNormal;

public:

	PlaneConstraint(glm::vec3 _planeNormal);
	void setPlaneNormal(glm::vec3 _planeNormal);
	glm::vec3 calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent);
	void calcConstraintedPointReference(glm::vec3 unconstrainted, Segment& current, Segment& parent);
};

class HingeConstraint : public Constraint {
private:
	float radius;

public:
	HingeConstraint(float _radius);
	void setHingeRadius(float _radius);
	glm::vec3 calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent);
	void calcConstraintedPointReference(glm::vec3 unconstrainted, Segment& current, Segment& parent);
};
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
};

class PlaneConstraint : public Constraint {
private:
	glm::vec3 planeNormal;

public:

	PlaneConstraint(glm::vec3 _planeNormal);
	void setPlaneNormal(glm::vec3 _planeNormal);
	glm::vec3 calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent);
};

class HingeConstraint : public Constraint {
private:
	float radius;

public:
	HingeConstraint(float _radius);

	glm::vec3 calcConstraintedPoint(glm::vec3 unconstrainted, Segment current, Segment parent);
};
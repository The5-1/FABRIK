#pragma once
#include <glm/glm.hpp>

//Forward declaration (makes it necessary to give a pointer: Constraint* constraint;)
class Constraint;

class Segment
{
public:
	glm::vec3 startJoint;
	glm::vec3 endJoint;

	Constraint* constraint = NULL;

	float length;
	float basicLength;
	float piston = 5.0f;
	float basicPiston = 5.0f;

	float angle;
public:
	Segment();
	Segment(glm::vec3 _startJoint, float _length, float _angle);
	Segment(glm::vec3 _startJoint, float _length, float _angle, Constraint* _constraint);
	Segment(glm::vec3 _startJoint, glm::vec3 _endJoint);
	Segment(glm::vec3 _startJoint, glm::vec3 _endJoint, Constraint* _constraint);
	~Segment();

	void setAll(glm::vec3 _startJoint, float _length, float _angle);

	void calculateEndJoint();
	void update();

};


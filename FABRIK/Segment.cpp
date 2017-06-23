#include "Segment.h"
#include "Constraint.h"

/*
* Default construcot for Segment (does not get any input values), is needed to initiate tree with type segment: 
*tree<Segment> segmentTree; (see tree.hh)
*/
Segment::Segment()
{

}

Segment::Segment(glm::vec3 _startJoint, float _length, float _angle)
{
	this->startJoint = _startJoint;
	this->length = _length;
	this->basicLength = _length;
	this->angle = _angle;

	this->update();
}

Segment::Segment(glm::vec3 _startJoint, float _length, float _angle, Constraint * _constraint)
{
	this->startJoint = _startJoint;
	this->length = _length;
	this->basicLength = _length;
	this->angle = _angle;
	this->constraint = _constraint;
	this->update();
}

Segment::Segment(glm::vec3 _startJoint, glm::vec3 _endJoint) {
	this->startJoint = _startJoint;
	this->endJoint = _endJoint;
	this->length = glm::length(_startJoint - _endJoint);
	this->basicLength = this->length;
}

Segment::Segment(glm::vec3 _startJoint, glm::vec3 _endJoint, Constraint * _constraint)
{
	this->startJoint = _startJoint;
	this->endJoint = _endJoint;
	this->constraint = _constraint;
	this->length = glm::length(_startJoint - _endJoint);
	this->basicLength = this->length;
}

Segment::~Segment()
{
}

void Segment::setAll(glm::vec3 _startJoint, float _length, float _angle)
{
	this->startJoint = _startJoint;
	this->length = _length;
	this->angle = _angle;
	this->basicLength = this->length;
}

void Segment::calculateEndJoint()
{
	float dx = length * glm::cos(angle);
	//float dy = length * glm::sin(angle);
	float dz = length * glm::sin(angle);
	endJoint = glm::vec3(startJoint.x + dx, 0, startJoint.z + dz);
}

void Segment::update() {
	this->calculateEndJoint();
}

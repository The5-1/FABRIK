#pragma once

#include "tree.hh"
#include "Segment.h"
#include "helper.h"
#include "Constraint.h"

#include <vector>
#include <glm/glm.hpp>

#define M_PI 3.14159265359

class FABRIK
{
public:

private:
	tree<Segment> chain;
	std::vector<glm::vec3> targets;
	int nrLeafs;
	//ToDo: Ändere die vererbung in helper.h	von:	class solidSphere
	//											zu:		class solidSphere : public simpleModel 
	//Somit können die Modelle nun beliebig übergeben werden z.b.: 
	//simpleModel jointModel = new solidSphere(5, 20, 20);
	//simpleModel boneModel = new solidCone(5, 20, 20);;

	solidSphere* jointModel = new solidSphere(5, 20, 20);
	
	solidCylinder* boneModel = new solidCylinder(20);
	
	//solidCone* coneConstraintModel = new solidCone(20);
	solidCone* coneConstraintModel = new solidCone(M_PI / 6, 20, glm::vec3(0.0f, 1.0f, 0.0f), 1);

	solidCircle* circleConstraintModel = new solidCircle(10);

public:
	FABRIK(tree<Segment> _chain, std::vector<glm::vec3> _targets);
	~FABRIK();

	

	//Utility
	void changeConstraints(Constraint* _constraint);
	void updateNrLeafs();

	//Draw
	void drawChain(const int program, const float radius, glm::mat4 V, glm::mat4 P);
	void FABRIK::drawHingeConstraints(const int program, glm::mat4 V, glm::mat4 P, float radius);
	void FABRIK::drawConstraints(const int program, glm::mat4 V, glm::mat4 P);

	//Calculation
	void updateChain(vector<glm::vec3> targets);
	void updatePistonChain(vector<glm::vec3> targets, int iterations);
	void updateHingeChain(vector<glm::vec3> targets, float radius);
	void updateChainWithConstraints(vector<glm::vec3> targets);

	//Full Pack
	void updateAndDraw(vector<glm::vec3> targets, const int program, const float radius, glm::mat4 V, glm::mat4 P);
private:
};


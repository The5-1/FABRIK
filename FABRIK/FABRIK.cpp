#include "FABRIK.h"
#include "mathUtility.h"


FABRIK::FABRIK(tree<Segment> _chain, std::vector<glm::vec3> _targets)
{
	chain = _chain;
	targets = _targets;
	this->updateNrLeafs();

	jointModel->upload();
	boneModel->upload();
	coneConstraintModel->upload();
	circleConstraintModel->upload();
}

FABRIK::~FABRIK()
{
}

void FABRIK::changeConstraints(Constraint* _constraint) {
	tree<Segment>::pre_order_iterator br = chain.begin();
	while (br != chain.end(chain.begin())) {
		br->constraint = _constraint;
		++br;
	}
}

void FABRIK::updateNrLeafs()
{	
	nrLeafs = 0;
	tree<Segment>::leaf_iterator sib = chain.begin_leaf();
	while (sib != chain.end_leaf()) {
		nrLeafs++;
		++sib;
	}
}

void FABRIK::drawChain(const int program, const float radius, glm::mat4 V, glm::mat4 P) {
	glUseProgram(program);
	uniform(program, "viewMatrix", V);
	uniform(program, "projMatrix", P);

	tree<Segment>::breadth_first_iterator br = chain.begin();
	while (br != chain.end(chain.begin())) {
		uniform(program, "color", glm::vec3(0, 1, 0));
		uniform(program, "modelMatrix", glm::translate(br->startJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));
		jointModel->draw();

		if (br.number_of_children() == 0) {
			uniform(program, "color", glm::vec3(0, 1, 0));
			uniform(program, "modelMatrix", glm::translate(br->endJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));
			jointModel->draw();
		}

		++br;
	}

	br = chain.begin();
	int counter = 0;
	while (br != chain.end(chain.begin())) {
		uniform(program, "color", glm::vec3(0, 0, 1));

		glm::vec3 segmentAxis = br->endJoint - br->startJoint;

		glm::mat3x3 r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

		glm::mat4 translate = glm::translate( br->startJoint );

		glm::mat4 rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);

		rotation = glm::transpose(rotation);

		uniform(program, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, glm::length(segmentAxis), 0.1f)));

		boneModel->draw();

		++br;
	}

	glUseProgram(0);
}

void FABRIK::drawHingeConstraints(const int program, glm::mat4 V, glm::mat4 P, float radius) {
	glUseProgram(program);
	uniform(program, "viewMatrix", V);
	uniform(program, "projMatrix", P);

	uniform(program, "color", glm::vec3(1.0f, 0.5f, 0));

	tree<Segment>::pre_order_iterator br = chain.begin();
	while (br != chain.end(chain.begin())) {
		if (chain.parent(br) == NULL) {
		}
		else {
			//Draw Constraint
			glm::vec3 segmentAxis = chain.parent(br)->endJoint - chain.parent(br)->startJoint;
			//glm::vec3 segmentAxis = br->endJoint - br->startJoint;

			glm::mat3x3 r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

			glm::mat4 translate = glm::translate(br->startJoint);

			glm::mat4 scale = glm::scale(glm::vec3(2*radius));

			glm::mat4 rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
				r[1][0], r[1][1], r[1][2], 0,
				r[2][0], r[2][1], r[2][2], 0,
				0, 0, 0, 1);

			rotation = glm::transpose(rotation);

			uniform(program, "modelMatrix", translate*rotation*scale);

			// Turn on wireframe mode
			glPolygonMode(GL_FRONT, GL_LINE);
			glPolygonMode(GL_BACK, GL_LINE);

			// Draw the box
			circleConstraintModel->draw();

			// Turn off wireframe mode
			glPolygonMode(GL_FRONT, GL_FILL);
			glPolygonMode(GL_BACK, GL_FILL);
		}
		++br;
	}
	glUseProgram(0);
}

void FABRIK::drawConstraints(const int program, glm::mat4 V, glm::mat4 P) {
	glUseProgram(program);
	uniform(program, "viewMatrix", V);
	uniform(program, "projMatrix", P);

	uniform(program, "color", glm::vec3(1.0f, 0.5f, 0));

	tree<Segment>::pre_order_iterator br = chain.begin();
	while (br != chain.end(chain.begin())) {
		if (chain.parent(br) == NULL) {
		}
		else {
			//Draw Constraint
			glm::vec3 segmentAxis = chain.parent(br)->endJoint - chain.parent(br)->startJoint;
			//glm::vec3 segmentAxis = br->endJoint - br->startJoint;

			glm::mat3x3 r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

			glm::mat4 translate = glm::translate(br->startJoint);

			glm::mat4 rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
				r[1][0], r[1][1], r[1][2], 0,
				r[2][0], r[2][1], r[2][2], 0,
				0, 0, 0, 1);

			rotation = glm::transpose(rotation);

			uniform(program, "modelMatrix", translate*rotation);

			// Turn on wireframe mode
			glPolygonMode(GL_FRONT, GL_LINE);
			glPolygonMode(GL_BACK, GL_LINE);

			// Draw the box
			coneConstraintModel->draw();

			// Turn off wireframe mode
			glPolygonMode(GL_FRONT, GL_FILL);
			glPolygonMode(GL_BACK, GL_FILL);
		}
		++br;
	}
	glUseProgram(0);
}

void FABRIK::updateChain(vector<glm::vec3> targets) {
	glm::vec3 start = chain.begin()->startJoint;
	int targetNr = 0;

	//Forwards-Step
	tree<Segment>::leaf_iterator sibParent = chain.begin_leaf();
	while (sibParent != chain.end_leaf()) {

		tree<Segment>::iterator parent = sibParent;
		while (parent != NULL) {
			parent->endJoint = targets[targetNr];
			parent->startJoint = parent->length * glm::normalize(parent->startJoint - parent->endJoint) + parent->endJoint;

			targets[targetNr] = parent->startJoint;

			parent = chain.parent(parent);
		}

		targetNr++;
		++sibParent;
	}

	//Backwards-Step
	tree<Segment>::pre_order_iterator br = chain.begin();
	while (br != chain.end(chain.begin())) {
		if (chain.parent(br) == NULL) {
		}
		else {
			start = chain.parent(br)->endJoint;
		}
		br->startJoint = start;
		br->endJoint = br->length * glm::normalize(br->endJoint - br->startJoint) + br->startJoint;
		++br;
	}
}

void FABRIK::updatePistonChain(vector<glm::vec3> targets, int iterations) {

	/*float epsilon = 0.1;*/
	float epsilon = 0.0f;


	std::cout << "Piston: " << std::endl;

	glm::vec3 start = chain.begin()->startJoint;
	int targetNr = 0;

	//Check if target reachable (use pistons)
	//Reset to base length
	tree<Segment>::pre_order_iterator iter = chain.begin();

	while (iter != chain.end(chain.begin())) {
		iter->length = iter->basicLength;
		iter->piston = iter->basicPiston;
		++iter;
	}

	//Update the chain once with FABRIK
	updateChain(targets);

	for (int i = 0; i < 1; i++) {
		float distanceToTarget;
		float currentLength;
		iter = chain.begin();
		tree<Segment>::pre_order_iterator iterStart = chain.begin();
		tree<Segment>::pre_order_iterator iterEnd;

		while (iter != chain.end(chain.begin())) {

			if (chain.child(iter, 0) == NULL) {
				distanceToTarget = glm::length(iter->endJoint - targets[targetNr]);
				iterEnd = iter;
				
				if (distanceToTarget > epsilon) {
					while (chain.child(iterStart, 0) != NULL) {
						//This chain will stop once we reach the leaf (there seems to be NO <= operation for iterators)
						//To Fix this this code is continued at (*)
						float clampedDistance = glm::clamp(distanceToTarget, 0.0f, iterStart->piston);
						distanceToTarget -= clampedDistance;
						iterStart->piston -= clampedDistance;
						iterStart->length += clampedDistance;
						std::cout << "iterStart->length: " << iterStart->length << std::endl;
						++iterStart;
					}

					//Fix from (*)
					float clampedDistance = glm::clamp(distanceToTarget, 0.0f, iterStart->piston);
					iterStart->piston -= clampedDistance;
					iterStart->length += clampedDistance;
					std::cout << "iterStart->length: " << iterStart->length << std::endl;
				}

				updateChain(targets);

				++iter;
				iterStart = iter;
			}
			else {
				updateChain(targets);
				++iter;
			}
		}

		//Update FABRIK with the new lengths
		updateChain(targets);
	}
}

void FABRIK::updateHingeChain(vector<glm::vec3> targets, float radius) {
	glm::vec3 start = chain.begin()->startJoint;
	int targetNr = 0;

	tree<Segment>::pre_order_iterator br = chain.begin();
	while (br != chain.end(chain.begin())) {
		if (chain.parent(br) == NULL) {
		}
		else {
			br->startJoint = chain.parent(br)->endJoint;
		}
		++br;
	}


	//Forwards-Step
	targetNr = 0;
	tree<Segment>::leaf_iterator sibParent = chain.begin_leaf();
	while (sibParent != chain.end_leaf()) {

		tree<Segment>::iterator parent = sibParent;
		while (parent != NULL) {
			parent->endJoint = targets[targetNr];
			parent->startJoint = parent->length * glm::normalize(parent->startJoint - parent->endJoint) + parent->endJoint;

			targets[targetNr] = parent->startJoint;

			parent = chain.parent(parent);
		}

		targetNr++;
		++sibParent;
	}

	//Backwards-Step
	br = chain.begin();
	while (br != chain.end(chain.begin())) {
		if (chain.parent(br) == NULL) {
		}
		else {
			start = chain.parent(br)->endJoint;
		}

		
		br->startJoint = start;
		br->endJoint = br->length * glm::normalize(br->endJoint - br->startJoint) + br->startJoint;
		
		//Apply hinge constraint
		if (chain.parent(br) != NULL) {
			float maxRadius = radius;
			glm::vec3 currentVector = br->endJoint - br->startJoint;
			glm::vec3 planeNormal = glm::normalize(chain.parent(br)->endJoint - chain.parent(br)->startJoint);
			//glm::vec3 planeNormal = chain.parent(br)->startJoint - chain.parent(br)->endJoint;

			glm::vec3 projectionOnPlane = currentVector - glm::dot(currentVector, planeNormal)*planeNormal;
			glm::vec3 projectionPoint = projectionOnPlane + br->startJoint;
			
			//float length = glm::length(br->startJoint - projectionPoint);
			float length = glm::length(projectionOnPlane);

			if (length > maxRadius) {
				projectionOnPlane = maxRadius * glm::normalize(projectionOnPlane);
			}

			br->startJoint = br->startJoint + projectionOnPlane;

		}
		++br;
	}
}

void FABRIK::updateChainWithConstraints(vector<glm::vec3> targets) {
	glm::vec3 start = chain.begin()->startJoint;
	int targetNr = 0;

	//Forwards-Step
	tree<Segment>::leaf_iterator sibParent = chain.begin_leaf();
	while (sibParent != chain.end_leaf()) {

		tree<Segment>::iterator parent = sibParent;
		while (parent != NULL) {
			parent->endJoint = targets[targetNr];
			parent->startJoint = parent->length * glm::normalize(parent->startJoint - parent->endJoint) + parent->endJoint;

			targets[targetNr] = parent->startJoint;

			parent = chain.parent(parent);
		}

		targetNr++;
		++sibParent;
	}

	//Backwards-Step
	tree<Segment>::pre_order_iterator br = chain.begin();
	tree<Segment>::iterator parentBr;

	while (br != chain.end(chain.begin())) {

		parentBr = chain.parent(br);

		if (parentBr == NULL) {
			//We are at the root.
		}
		else {
			start = parentBr->endJoint;
		}

		br->startJoint = start;
		br->endJoint = br->length * glm::normalize(br->endJoint - br->startJoint) + br->startJoint;

		if (br->constraint != NULL && parentBr != NULL)
		{
			br->endJoint = br->constraint->calcConstraintedPoint(br->endJoint, *br, *parentBr);
		}
		
		++br;
	}
}

void FABRIK::updateAndDraw(vector<glm::vec3> targets, const int program, const float radius, glm::mat4 V, glm::mat4 P)
{
	//Placeholder-Function to speed up Testing

	////this->updatePistonChain(targets, 10);
	////this->updateChainWithConstraints(targets);
	//this->updateHingeChain(targets);
	//this->drawChain(program, radius, V, P);
	//
	////this->drawConstraints(program, V, P);
	//this->drawHingeConstraints(program, V, P);
}

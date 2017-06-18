//--------- Includes --------------
#include <iostream>
#include <math.h>
#include <AntTweakBar.h>

#include <glew.h>
#include <glut.h>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>

#include "helper.h"
#include "mathUtility.h"

//Includes for FABRIK
#include "FABRIK.h"
#include "tree.hh"
#include "tree_util.hh"


//Globals for FABRIK
std::vector<Segment> segments;
vector<glm::vec3> targets(5);

//defines
#define M_PI 3.14159265359

// globals
float rotationAngle = 0.0f;
float degreesPerSecond = 1.f;
bool autoRotate =  true;
float radius = 8.0f;

float sphereHeight = 10;
bool solveIK = false;
int stepsIK = 10;
bool writeOnceToConsole = true;

bool fabrikSolver = true;
bool fabrikConeSolver = false;
bool fabrikPlaneSolver = false;
bool fabrikHingeSolver = false;
bool fabrikPistonSolver = false;
bool updateChain = true;
tree<Segment> segmentTree;

cameraSystem cam(1.0f, 1.0f, glm::vec3(3.0f, 16.0f, 22.f));
Timer timer;

// render objects
solidSphere *sphere = 0;
solidCone *cone = 0;
groundPlane *plane = 0;
solidCylinder *cylinder = 0;
triangleList *tList = 0;
triangleList *tListTree = 0;

vector<glm::vec3> ropeTriangles;
vector<glm::vec3> treeTriangles;

// shaders
GLuint gbufferShader;
GLuint basicShader;


glm::mat4 viewMatrix;
glm::mat4 projMatrix;
glm::vec3 lightDir;

// frame buffer object
Fbo *fbo = 0;

// textures
Tex *diffuse = 0, *normal = 0, *position = 0, *depth = 0;

// tweak bar
TwBar *tweakBar;
bool drawRope = false;
bool drawTree = false;
bool useConstraints = true;

//FABRIK
FABRIK *fabrik = 0;
//Constraints Value
float angleCone = M_PI / 6;
glm::vec3 planeNormal = glm::vec3(1.0f, 0.0f, 0.0f);
float radiusHinge = 1.0f;
//Constraints
Constraint *constraintCone = new ConeConstraint(M_PI / 6);
Constraint *constraintPlane = new PlaneConstraint(planeNormal);

void setupTweakBar() {
	TwInit(TW_OPENGL_CORE, NULL);
	tweakBar = TwNewBar("Settings");
	//TwBar *bar;

	TwAddVarRW(tweakBar, "No constraint", TW_TYPE_BOOLCPP, &fabrikSolver, " label='No constraint' ");
	TwAddVarRW(tweakBar, "Cone", TW_TYPE_BOOLCPP, &fabrikConeSolver, " label='Cone' ");
	TwAddVarRW(tweakBar, "Plane", TW_TYPE_BOOLCPP, &fabrikPlaneSolver, " label='Plane' ");
	TwAddVarRW(tweakBar, "Hinge", TW_TYPE_BOOLCPP, &fabrikHingeSolver, " label='Hinge' ");
	TwAddVarRW(tweakBar, "Piston", TW_TYPE_BOOLCPP, &fabrikPistonSolver, " label='Piston' ");
	//TwAddVarRW(tweakBar, "Draw rope", TW_TYPE_BOOLCPP, &drawRope, " label='Draw rope' ");
	//TwAddVarRW(tweakBar, "Draw tree", TW_TYPE_BOOLCPP, &drawTree, " label='Draw tree' ");
	//TwAddVarRW(tweakBar, "Use constraints", TW_TYPE_BOOLCPP, &useConstraints, " label='Use constraints' ");
	//TwAddVarRW(tweakBar, "solveIK", TW_TYPE_BOOLCPP, &solveIK, " label='Solve IK' ");
	TwAddVarRW(tweakBar, "Update chain", TW_TYPE_BOOLCPP, &updateChain, " label='Update chain' ");

	TwAddVarRW(tweakBar, "planeNormal", TW_TYPE_DIR3F, &planeNormal, " label='Plane Normal'");

	TwAddVarRW(tweakBar, "radiusHinge", TW_TYPE_FLOAT, &radiusHinge, " label='radius Hinge' min=0 step=0.1 max=100 ");

	TwAddVarRW(tweakBar, "stepsIK", TW_TYPE_INT32, &stepsIK, " label='Steps for IK' ");

	TwAddVarRW(tweakBar, "Rotate", TW_TYPE_BOOLCPP, &autoRotate, " label='Rotate Target' ");
	TwAddVarRW(tweakBar, "RotSpeed", TW_TYPE_FLOAT, &degreesPerSecond, " label='Rotation Speed' min=0 step=0.1 max=360 ");
	TwAddVarRW(tweakBar, "SphereHeight", TW_TYPE_FLOAT, &sphereHeight, " label='Target height' min=0 step=0.1 max=100 ");

	//TwAddVarRW(tweakBar, "lightDirection", TW_TYPE_DIR3F, &lightDir, "label='Light Direction'");
}

void initiateTestTree() {
	Constraint *constraint = NULL;

	tree<Segment>::iterator root = segmentTree.insert(segmentTree.begin(), Segment(glm::vec3(0, 0, 0), 3, glm::radians(-90.0f), constraint));
	root->update();

	tree<Segment>::iterator first = segmentTree.append_child(root, Segment(root->endJoint, 3, glm::radians(-90.0f), constraint));
	first->update();

	tree<Segment>::iterator second = segmentTree.append_child(first, Segment(first->endJoint, 3, glm::radians(-135.0f), constraint));
	second->update();

	tree<Segment>::iterator third = segmentTree.append_child(first, Segment(first->endJoint, 3, glm::radians(-45.0f), constraint));
	third->update();

	Segment leaf1 = Segment(second->endJoint, 2, glm::radians(-135.0f), constraint);
	leaf1.update();
	segmentTree.append_child(second, leaf1);

	Segment leaf2 = Segment(second->endJoint, 4, glm::radians(-90.0f), constraint);
	leaf2.update();
	segmentTree.append_child(second, leaf2);

	Segment leaf3 = Segment(second->endJoint, 2, glm::radians(-45.0f), constraint);
	leaf3.update();
	segmentTree.append_child(second, leaf3);

	Segment leaf4 = Segment(third->endJoint, 4, glm::radians(-100.0f), constraint);
	leaf4.update();
	segmentTree.append_child(third, leaf4);

	Segment leaf5 = Segment(third->endJoint, 4, glm::radians(-60.0f), constraint);
	leaf5.update();
	segmentTree.append_child(third, leaf5);
}

//void drawTestTree(const int program, const float radius) {
//	glUseProgram(program);
//	uniform(program, "viewMatrix", viewMatrix);
//	uniform(program, "projMatrix", projMatrix);
//
//	tree<Segment>::breadth_first_iterator br = segmentTree.begin();
//	while (br != segmentTree.end(segmentTree.begin())) {
//		uniform(program, "color", glm::vec3(0, 1, 0));
//		uniform(program, "modelMatrix", glm::translate(br->startJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));
//		sphere->draw();
//
//		if (br.number_of_children() == 0) {
//			uniform(program, "color", glm::vec3(0, 1, 0));
//			uniform(program, "modelMatrix", glm::translate(br->endJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));
//				
//			sphere->draw();
//		}
//
//		++br;
//	}
//	glUseProgram(0);
//
//	vector<glm::vec3> triangleJoints;
//
//	br = segmentTree.begin();
//	int counter = 0;
//	while (br != segmentTree.end(segmentTree.begin())) {
//		glm::vec4 homogenTarget = (glm::translate(br->startJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius))) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
//		triangleJoints.push_back(glm::vec3(homogenTarget.x, homogenTarget.y, homogenTarget.z));
//
//		homogenTarget = (glm::translate(br->endJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius))) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
//		triangleJoints.push_back(glm::vec3(homogenTarget.x, homogenTarget.y, homogenTarget.z));
//
//		triangleJoints.push_back(0.5f*(triangleJoints[3 * counter] + triangleJoints[3 * counter]) + glm::vec3(0.0f, 0.2f, 0.0f));
//
//		counter++;
//		++br;
//	}
//	treeTriangles = triangleJoints;
//}
//
//void updateTestTree(vector<glm::vec3> targets) {
//	glm::vec3 start = segmentTree.begin()->startJoint;
//	int targetNr = 0;
//
//	//Forwards-Step
//	tree<Segment>::leaf_iterator sibParent = segmentTree.begin_leaf();
//	while (sibParent != segmentTree.end_leaf()) {
//
//		tree<Segment>::iterator parent = sibParent;
//		while (parent != NULL) {
//			parent->endJoint = targets[targetNr];
//			parent->startJoint = parent->length * glm::normalize(parent->startJoint - parent->endJoint) + parent->endJoint;
//
//			targets[targetNr] = parent->startJoint;
//
//			parent = segmentTree.parent(parent);
//		}
//
//		targetNr++;
//		++sibParent;
//	}
//
//	//Backwards-Step
//	tree<Segment>::pre_order_iterator br = segmentTree.begin();
//	while (br != segmentTree.end(segmentTree.begin())) {
//		if (segmentTree.parent(br) == NULL) {
//		}
//		else {
//			start = segmentTree.parent(br)->endJoint;
//		}
//		br->startJoint = start;
//		br->endJoint = br->length * glm::normalize(br->endJoint - br->startJoint) + br->startJoint;
//		++br;
//	}
//}
//
//void initiateRope() {	
//	segments.push_back(Segment(glm::vec3(0, 0, 0), 5, glm::radians(45.0f)));
//	segments[0].update();
//
//	segments.push_back(Segment(segments[0].endJoint, 5, 0));
//	segments[1].update();
//
//	segments.push_back(Segment(segments[1].endJoint, 5, glm::radians(33.0f)));
//	segments[2].update();
//}
//
//void updateRope(glm::vec3 target) {
//	int numberSegments = segments.size();
//	glm::vec3 start = segments[0].startJoint;
//
//	//Distance-Criteria
//	float sumDistance = 0;
//	for (int i = 0; i <= numberSegments - 1; i++) {
//		sumDistance += segments[i].length;
//	}
//
//	//If Distance-Criteria fails, put joints on line to target
//	if (sumDistance < glm::length(segments[0].startJoint - target)) {
//		glm::vec3 direction = glm::normalize(target - segments[0].startJoint);
//
//		segments[0].endJoint = segments[0].startJoint + segments[0].length*direction;
//
//		for (int i = 1; i <= numberSegments - 1; i++) {
//			segments[i].startJoint = segments[i - 1].endJoint;
//			segments[i].endJoint = segments[i].startJoint + segments[i].length*direction;
//		}
//
//		return;
//	}
//
//	//Distance-Criteria succeeds
//	//Forward-Step
//	for (int i = numberSegments - 1; i >= 0; i--) {
//		segments[i].endJoint = target;
//		segments[i].startJoint = segments[i].length * glm::normalize(segments[i].startJoint - segments[i].endJoint) + segments[i].endJoint;
//		target = segments[i].startJoint;
//	}
//
//	//Backwards-Step
//	for (int i = 0; i < numberSegments; i++) {
//		
//		if (i != 0 && useConstraints) {
//			//ToDo: Cones in segments speichern
//			float coneAngle = M_PI / 6;
//
//			glm::vec3 d = (segments[i - 1].endJoint - segments[i - 1].startJoint);
//			//glm::vec3 d = glm::vec3(0.0f, 1.0f, 0.0f);
//
//
//			segments[i].startJoint = start;
//			segments[i].endJoint = segments[i].length * glm::normalize(segments[i].endJoint - segments[i].startJoint) + segments[i].startJoint;
//			
//
//			//Sicherheitskopie
//			/*
//			//FABRIK
//			segments[i].constraint = new ConeConstraint(M_PI / 6);
//			start = segments[i].constraint->calcConstraintedPoint(segments[i].endJoint, segments[i], segments[i - 1]);
//			*/
//
//			//Rotation
//			glm::mat3x3 rotation = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), d);
//			d = rotation * glm::normalize(d);
//			//d = glm::vec3(0.0f, 1.0f, 0.0f);
//			//std::cout << "d ( " << d.x << ", " << d.y << ", " << d.z << ")" << std::endl;
//
//			//Projection
//			//glm::vec3 c = (segments[i].endJoint - segments[i].startJoint);
//			glm::vec3 c = rotation * (segments[i].endJoint - segments[i].startJoint);
//			//std::cout << "c ( " << c.x << ", " << c.y << ", " << c.z << ")" << std::endl;
//			float height = c.y;
//			//std::cout << height << std::endl;
//			float radius = height * glm::tan(coneAngle);
//
//			glm::vec3 p = glm::dot(c, d) * d;
//			//std::cout << "p ( " << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
//
//			if (glm::dot(p, d) < 0) {
//				p = -p;
//			}
//
//			glm::vec3 z = c - p;
//			//std::cout << "z ( " << z.x << ", " << z.y << ", " << z.z << ")" << std::endl;
//
//			if ((z.x*z.x) / (radius*radius) + (z.z*z.z) / (radius*radius) > 1) {
//				//std::cout << " Outside of ellipse " << std::endl;
//				float sigma = atan2(z.z, z.x);
//				//float sigma = atan2(segments[i].endJoint.z, segments[i].endJoint.x);
//				//float sigma = atan2(segments[i].endJoint.x, segments[i].endJoint.z);
//
//				p.x = radius * glm::cos(sigma);
//				p.y = glm::abs(height);
//				p.z = radius * glm::sin(sigma);
//
//				p = transpose(rotation) * p;
//				//std::cout << "p ( " << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
//				segments[i].endJoint = segments[i].startJoint + segments[i].length * glm::normalize(p);
//			}
//			
//			start = segments[i].endJoint;
//		}
//		else {
//			segments[i].startJoint = start;
//			segments[i].endJoint = segments[i].length * glm::normalize(segments[i].endJoint - segments[i].startJoint) + segments[i].startJoint;
//			start = segments[i].endJoint;
//		}
//	}
//}
//
//void drawCycleSegments(const int program, const float radius) {
//
//	glUseProgram(program);
//	uniform(program, "viewMatrix", viewMatrix);
//	uniform(program, "projMatrix", projMatrix);
//
//	for (int i = 0; i < segments.size(); ++i) {
//		uniform(program, "color", glm::vec3(1, 0, 0));
//		uniform(program, "modelMatrix", glm::translate(segments[i].startJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));
//		sphere->draw();
//	}
//
//	uniform(program, "modelMatrix", glm::translate(segments[segments.size()-1].endJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));
//
//	sphere->draw();
//
//	glUseProgram(0);
//
//	//Draw Triangle between joints
//	vector<glm::vec3> triangleJoints;
//	for (int i = 0; i < segments.size(); i++) {
//		glm::vec4 homogenTarget = (glm::translate(segments[i].startJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius))) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
//		triangleJoints.push_back(glm::vec3(homogenTarget.x, homogenTarget.y, homogenTarget.z));
//
//		homogenTarget = (glm::translate(segments[i].endJoint) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius))) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
//		triangleJoints.push_back(glm::vec3(homogenTarget.x, homogenTarget.y, homogenTarget.z));
//
//		triangleJoints.push_back(0.5f*(triangleJoints[3*i] + triangleJoints[3 * i]) + glm::vec3(0.0f, 0.2f, 0.0f));	
//	}
//	ropeTriangles = triangleJoints;
//
//	//Draw Constraints
//	glUseProgram(program);
//	uniform(program, "viewMatrix", viewMatrix);
//	uniform(program, "projMatrix", projMatrix);
//
//	uniform(program, "color", glm::vec3(1.0f, 0.5f, 0));
//	for (int i = 0; i < triangleJoints.size()/3 - 1; i++) {
//		
//		glm::vec3 segmentAxis = triangleJoints[3 * i + 1] - triangleJoints[3 * i];
//
//		glm::mat3x3 r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);
//
//		glm::mat4 translate = glm::translate(triangleJoints[3 * i + 1]);
//
//		glm::mat4 rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
//										r[1][0], r[1][1], r[1][2], 0,
//										r[2][0], r[2][1], r[2][2], 0,
//										0, 0, 0, 1);
//
//		rotation = glm::transpose(rotation);
//
//		uniform(program, "modelMatrix", translate*rotation);
//
//		// Turn on wireframe mode
//		glPolygonMode(GL_FRONT, GL_LINE);
//		glPolygonMode(GL_BACK, GL_LINE);
//		// Draw the box
//		cone->draw();
//		// Turn off wireframe mode
//		glPolygonMode(GL_FRONT, GL_FILL);
//		glPolygonMode(GL_BACK, GL_FILL);
//	}
//	glUseProgram(0);
//}



float splatDistFromOrigin() {
	return (cos(rotationAngle*0.7) + 1.0f) * 4.f + 1.0f;
}


void drawSplats(const int program, const float radius, const bool asLight) {

	glUseProgram(program);
	uniform(program, "viewMatrix", viewMatrix);
	uniform(program, "projMatrix", projMatrix);

	glm::vec3 splatColors[6] = { glm::vec3(1, 0, 0),
								 glm::vec3(1, 1, 0),
								 glm::vec3(0, 1, 0),
								 glm::vec3(0, 1, 1),
								 glm::vec3(0, 0, 1),
								 glm::vec3(1, 0, 1) };

	for (int i = 0; i < 5; ++i) {
		float rad = ((float)i / 6.f) * M_PI * 2 + rotationAngle; 
		float dist = splatDistFromOrigin();
		float x = cos(rad)*dist;
		float y = sin(rad)*dist;

		uniform(program, "color", splatColors[i]);
		uniform(program, "modelMatrix", glm::translate(glm::vec3(x, sphereHeight, y)) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));

		glm::vec4 homogenTarget = (glm::translate(glm::vec3(x, sphereHeight, y)) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius))) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	
		targets[i] = glm::vec3(homogenTarget.x, homogenTarget.y, homogenTarget.z);

		sphere->draw();
	}
	glUseProgram(0);
}

//void updateTestTriangle(vector<glm::vec3> triangles) {
//	tList = new triangleList(triangles);
//	tList->upload();
//}
//
//void updateTestTriangle2(vector<glm::vec3> triangles) {
//	tListTree = new triangleList(triangles);
//	tListTree->upload();
//}

void init() {
	glEnable(GL_TEXTURE_2D);
	fbo = new Fbo("DR", WIDTH, HEIGHT, 3);
	gl_check_error("fbo");
	diffuse = new Tex(WIDTH, HEIGHT, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("diffuse tex");
	normal = new Tex(WIDTH, HEIGHT, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("normal tex");
	position = new Tex(WIDTH, HEIGHT, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("position tex");
	depth = new Tex(WIDTH, HEIGHT, GL_DEPTH_COMPONENT32, GL_DEPTH_COMPONENT, GL_FLOAT);
	gl_check_error("depth tex");

	// skipped tex image stuff

	fbo->Bind();
	glActiveTexture(GL_TEXTURE0);
	diffuse->Bind();
	fbo->AddTextureAsColorbuffer("diffuse", diffuse);
	fbo->AddTextureAsColorbuffer("normal", normal);
	fbo->AddTextureAsColorbuffer("position", position);
	fbo->AddTextureAsDepthbuffer(depth);
	fbo->Check();
	fbo->Unbind();
	gl_check_error("post fbo setup");

	plane = new groundPlane(0.f, 12.f);
	plane->upload();

	sphere = new solidSphere(5, 20, 20);
	sphere->upload();

	cone = new solidCone(M_PI / 6, 25, glm::vec3(0.0f, 1.0f, 0.0f), 5.0f);
	cone->upload();

	cylinder = new solidCylinder(24);
	cylinder->upload();

	//Fabrik
	/*initiateRope();*/
	initiateTestTree();
	fabrik = new FABRIK(segmentTree, targets);
}

void display () {
	
	//updateTestTriangle(ropeTriangles);
	//updateTestTriangle2(treeTriangles);

	timer.update();
	if (autoRotate)
		rotationAngle += timer.intervall*degreesPerSecond;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);


	// shade with primary light source
	
	glClearColor(0.2f, 0.2f, 0.2f, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glUseProgram(gbufferShader);
	uniform(gbufferShader, "color", glm::vec3(0.8f, 0.8f, 0.8f));
	uniform(gbufferShader, "viewMatrix", viewMatrix);
	uniform(gbufferShader, "projMatrix", projMatrix);
	uniform(gbufferShader, "modelMatrix", glm::mat4());

	// Turn on wireframe mode
	glPolygonMode(GL_FRONT, GL_LINE);
	glPolygonMode(GL_BACK, GL_LINE);
	// Draw the box
	plane->draw();
	// Turn off wireframe mode
	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_FILL);


	//if (drawRope == true) {
	//	tList->draw();
	//}
	//if (drawTree == true) {
	//	tListTree->draw();
	//}
	//glUseProgram(0);

	drawSplats(gbufferShader, 0.1f, false);

	//if (solveIK) {
	//	
	//	if (writeOnceToConsole) {
	//		std::cout << "Before Using IK: " << std::endl;
	//		for (int i = 0; i < segments.size(); i++) {
	//			std::cout << "Segment: " << i << " start " << segments[i].startJoint.x << " " << segments[i].startJoint.y << " " << segments[i].startJoint.z << " " << std::endl;
	//			std::cout << "Segment: " << i << " end " << segments[i].endJoint.x << " " << segments[i].endJoint.y << " " << segments[i].endJoint.z << " " << std::endl;
	//			std::cout << "length: " << glm::length(segments[i].startJoint - segments[i].endJoint) << std::endl;
	//		}
	//	}

	//	for (int i = 0; i < stepsIK; i++) {
	//		/*updateRope(targets[0]);
	//		updateTestTree(targets);*/
	//	}

	//	if (writeOnceToConsole) {
	//		std::cout << std::endl;
	//		std::cout << "After Using IK: " << std::endl;
	//		for (int i = 0; i < segments.size(); i++) {
	//			std::cout << "Segment: " << i << " start " << segments[i].startJoint.x << " " << segments[i].startJoint.y << " " << segments[i].startJoint.z << " " << std::endl;
	//			std::cout << "Segment: " << i << " end " << segments[i].endJoint.x << " " << segments[i].endJoint.y << " " << segments[i].endJoint.z << " " << std::endl;
	//			std::cout << "length: " << glm::length(segments[i].startJoint - segments[i].endJoint) << std::endl;
	//		}
	//	}
	//	writeOnceToConsole = false;
	//}

	//if (drawRope == true) {
	//	/*drawCycleSegments(gbufferShader, 0.1f);*/
	//}

	//if (drawTree == true) {
	//	/*drawTestTree(gbufferShader, 0.1f);*/
	//}

	//FABRIK
	if (fabrikSolver) {
		if(updateChain)
			fabrik->updateChain(targets);
		fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
	}
	else if (fabrikConeSolver) {
		fabrik->changeConstraints(constraintCone);
		if (updateChain)
			fabrik->updateChainWithConstraints(targets);
		fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
		fabrik->drawConstraints(gbufferShader, viewMatrix, projMatrix);
	}
	else if (fabrikPlaneSolver) {
		((PlaneConstraint*)constraintPlane)->setPlaneNormal(planeNormal);
		fabrik->changeConstraints(constraintPlane);
		if (updateChain)
			fabrik->updateChainWithConstraints(targets);
		fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
	}
	else if (fabrikPistonSolver) {
		if (updateChain)
			fabrik->updatePistonChain(targets, 10);
		fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
	}
	else if(fabrikHingeSolver){
		if (updateChain)
			fabrik->updateHingeChain(targets, radiusHinge);
		fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
		fabrik->drawHingeConstraints(gbufferShader, viewMatrix, projMatrix, radiusHinge);
	}
	else {
		fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
	}
	

	gl_check_error("after splatting");

	glDisable(GL_STENCIL_TEST);
	glDepthFunc(GL_LESS);

	depth->Unbind();
	normal->Unbind();
	diffuse->Unbind();

	glUseProgram(0);

	// tweak bar
	TwDraw();
	glutSwapBuffers();
	glutPostRedisplay();

}

void loadShader(bool init) {
	GLuint shader;
	if (createProgram_VF("gbuffer.vs.glsl", "gbuffer.fs.glsl", &shader))
		gbufferShader = shader;
	else if (init) exit(1);
	gl_check_error("gbuffer shader");

	std::cout << "shader loaded" << std::endl;
}

int main (int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STENCIL);

	glutCreateWindow("FABRIK");

	setupTweakBar();

	GLenum err = glewInit();
	if (GLEW_OK != err)
		std::cerr << "Error : " << glewGetErrorString(err) << std::endl;
	
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutMotionFunc(onMouseMove);
	glutMouseFunc(onMouseDown);
	glutReshapeFunc(reshape);
	glutIdleFunc(onIdle);

	TwGLUTModifiersFunc(glutGetModifiers);

	initGL();

	init();

	glutMainLoop();
	return 0;
}

		






 


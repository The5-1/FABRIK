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


//Model
typedef enum { TREE1, TREE2, TREE3, HAND } MESH_TYPE;
MESH_TYPE m_currentMesh = TREE1;
bool changeMesh = false;
//Target
typedef enum { TARGET1, TARGET2} TARGET_TYPE;
TARGET_TYPE m_currentTarget = TARGET1;
void setupTweakBar() {
	//WEB: http://ogldev.atspace.co.uk/www/tutorial48/tutorial48.html
	TwInit(TW_OPENGL_CORE, NULL);
	tweakBar = TwNewBar("Settings");
	

	// A variable for the current selection - will be updated by ATB
	
	// Array of drop down items
	TwEnumVal Meshes[] = { { TREE1, "Tree1" },{ TREE2, "Tree2" }, { TREE3, "Tree3" }, { HAND, "Hand" } };
	// ATB identifier for the array
	TwType MeshTwType = TwDefineEnum("MeshType", Meshes, 4);
	// Link it to the tweak bar
	TwAddVarRW(tweakBar, "Mesh", MeshTwType, &m_currentMesh, NULL);
	TwAddVarRW(tweakBar, "Change Mesh", TW_TYPE_BOOLCPP, &changeMesh, " label='Change Mesh' ");
	//Seperator
	TwAddSeparator(tweakBar, "", NULL);

	//Target
	TwEnumVal TargetsTw[] = { { TARGET1, "Target 1" },{ TARGET2, "Target 2" }};
	TwType TargetTwType = TwDefineEnum("Targets", TargetsTw, 2);
	TwAddVarRW(tweakBar, "Target", TargetTwType, &m_currentTarget, NULL);
	TwAddSeparator(tweakBar, "", NULL);


	TwAddVarRW(tweakBar, "No constraint", TW_TYPE_BOOLCPP, &fabrikSolver, " label='No constraint' ");
	TwAddVarRW(tweakBar, "Cone", TW_TYPE_BOOLCPP, &fabrikConeSolver, " label='Cone' ");
	TwAddVarRW(tweakBar, "Plane", TW_TYPE_BOOLCPP, &fabrikPlaneSolver, " label='Plane' ");
	TwAddVarRW(tweakBar, "Hinge", TW_TYPE_BOOLCPP, &fabrikHingeSolver, " label='Hinge' ");
	TwAddVarRW(tweakBar, "Piston", TW_TYPE_BOOLCPP, &fabrikPistonSolver, " label='Piston' ");
	//TwAddVarRW(tweakBar, "Draw rope", TW_TYPE_BOOLCPP, &drawRope, " label='Draw rope' ");
	//TwAddVarRW(tweakBar, "Draw tree", TW_TYPE_BOOLCPP, &drawTree, " label='Draw tree' ");
	//TwAddVarRW(tweakBar, "Use constraints", TW_TYPE_BOOLCPP, &useConstraints, " label='Use constraints' ");
	//TwAddVarRW(tweakBar, "solveIK", TW_TYPE_BOOLCPP, &solveIK, " label='Solve IK' ");

	TwAddSeparator(tweakBar, "", NULL);
	TwAddVarRW(tweakBar, "Update chain", TW_TYPE_BOOLCPP, &updateChain, " label='Update chain' ");

	TwAddVarRW(tweakBar, "planeNormal", TW_TYPE_DIR3F, &planeNormal, " label='Plane Normal'");

	TwAddVarRW(tweakBar, "radiusHinge", TW_TYPE_FLOAT, &radiusHinge, " label='radius Hinge' min=0 step=0.1 max=100 ");

	TwAddVarRW(tweakBar, "stepsIK", TW_TYPE_INT32, &stepsIK, " label='Steps for IK' ");
	TwAddSeparator(tweakBar, "", NULL);
	TwAddVarRW(tweakBar, "Rotate", TW_TYPE_BOOLCPP, &autoRotate, " label='Rotate Target' ");
	TwAddVarRW(tweakBar, "RotSpeed", TW_TYPE_FLOAT, &degreesPerSecond, " label='Rotation Speed' min=0 step=0.1 max=360 ");
	TwAddVarRW(tweakBar, "SphereHeight", TW_TYPE_FLOAT, &sphereHeight, " label='Target height' min=0 step=0.1 max=100 ");

	//TwAddVarRW(tweakBar, "lightDirection", TW_TYPE_DIR3F, &lightDir, "label='Light Direction'");
}

void initiateTestTree() {
	Constraint *constraint = NULL;
	tree<Segment> createTree;
	tree<Segment>::iterator root = createTree.insert(createTree.begin(), Segment(glm::vec3(0, 0, 0), 3, glm::radians(-90.0f), constraint));
	root->update();

	tree<Segment>::iterator first = createTree.append_child(root, Segment(root->endJoint, 3, glm::radians(-90.0f), constraint));
	first->update();

	tree<Segment>::iterator second = createTree.append_child(first, Segment(first->endJoint, 3, glm::radians(-135.0f), constraint));
	second->update();

	tree<Segment>::iterator third = createTree.append_child(first, Segment(first->endJoint, 3, glm::radians(-45.0f), constraint));
	third->update();

	Segment leaf1 = Segment(second->endJoint, 2, glm::radians(-135.0f), constraint);
	leaf1.update();
	createTree.append_child(second, leaf1);

	Segment leaf2 = Segment(second->endJoint, 4, glm::radians(-90.0f), constraint);
	leaf2.update();
	createTree.append_child(second, leaf2);

	Segment leaf3 = Segment(second->endJoint, 2, glm::radians(-45.0f), constraint);
	leaf3.update();
	createTree.append_child(second, leaf3);

	Segment leaf4 = Segment(third->endJoint, 4, glm::radians(-100.0f), constraint);
	leaf4.update();
	createTree.append_child(third, leaf4);

	Segment leaf5 = Segment(third->endJoint, 4, glm::radians(-60.0f), constraint);
	leaf5.update();
	createTree.append_child(third, leaf5);

	segmentTree = createTree;
}

void initiateTestTree2() {
	Constraint *constraint = NULL;
	tree<Segment> createTree;
	tree<Segment>::iterator root = createTree.insert(createTree.begin(), Segment(glm::vec3(0, 0, 0), 2, glm::radians(-90.0f), constraint));
	root->update();

	tree<Segment>::iterator first = createTree.append_child(root, Segment(root->endJoint, 2, glm::radians(-90.0f), constraint));
	first->update();

	tree<Segment>::iterator second = createTree.append_child(first, Segment(first->endJoint, 2, glm::radians(-135.0f), constraint));
	second->update();

	tree<Segment>::iterator third = createTree.append_child(second, Segment(second->endJoint, 2, glm::radians(-45.0f), constraint));
	third->update();

	Segment leaf1 = Segment(third->endJoint, 2, glm::radians(-135.0f), constraint);
	leaf1.update();
	createTree.append_child(third, leaf1);

	Segment leaf2 = Segment(third->endJoint, 4, glm::radians(-90.0f), constraint);
	leaf2.update();
	createTree.append_child(third, leaf2);

	Segment leaf3 = Segment(third->endJoint, 2, glm::radians(-45.0f), constraint);
	leaf3.update();
	createTree.append_child(third, leaf3);

	Segment leaf4 = Segment(third->endJoint, 4, glm::radians(-100.0f), constraint);
	leaf4.update();
	createTree.append_child(third, leaf4);

	Segment leaf5 = Segment(third->endJoint, 2, glm::radians(-60.0f), constraint);
	leaf5.update();
	createTree.append_child(third, leaf5);

	segmentTree = createTree;
}

void initiateTestTree3() {
	Constraint *constraint = NULL;
	tree<Segment> createTree;

	tree<Segment>::iterator root = createTree.insert(createTree.begin(), Segment(glm::vec3(0, 0, 0), glm::vec3(0, 2, 0), constraint));

	tree<Segment>::iterator firstFinger1 = createTree.append_child(root, Segment(root->endJoint, glm::vec3(0, 4, 0), constraint));
	tree<Segment>::iterator firstFinger2 = createTree.append_child(firstFinger1, Segment(firstFinger1->endJoint, glm::vec3(0, 6, 0), constraint));
	tree<Segment>::iterator firstFinger3 = createTree.append_child(firstFinger2, Segment(firstFinger2->endJoint, glm::vec3(0, 8, 0), constraint));

	Segment firstFinger4 = Segment(firstFinger3->endJoint, glm::vec3(0, 10, 0), constraint);
	createTree.append_child(firstFinger3, firstFinger4);
	segmentTree = createTree;
}

void handTree() {
	float resizeArm = 4.0f;
	float resizeFinger = 4.0f;
	Constraint *constraint = NULL;
	tree<Segment> createTree;

	tree<Segment>::iterator root = createTree.insert(createTree.begin(), Segment(glm::vec3(0, 0, 0), resizeArm * glm::vec3(0, 1, 0), constraint));

	tree<Segment>::iterator firstFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(-1.0, 1.5, 0), constraint));
	tree<Segment>::iterator secondFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(-0.5, 1.5, 0), constraint));
	tree<Segment>::iterator thirdFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(0, 1.5, 0), constraint));
	tree<Segment>::iterator fourthFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(0.5, 1.5, 0), constraint));
	//tree<Segment>::iterator fifthFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(0.5, 1.0, 0), constraint));
	tree<Segment>::iterator fifthFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(0.8, 1.0, 0), constraint));

	Segment firstFinger2 = Segment(firstFinger1->endJoint, resizeFinger*glm::vec3(-1.0, 2.5, 0), constraint);
	createTree.append_child(firstFinger1, firstFinger2);
	
	Segment secondFinger2 = Segment(secondFinger1->endJoint, resizeFinger*glm::vec3(-0.5, 2.5, 0), constraint);
	createTree.append_child(secondFinger1, secondFinger2);

	Segment thirdFinger2 = Segment(thirdFinger1->endJoint, resizeFinger*glm::vec3(0, 2.5, 0), constraint);
	createTree.append_child(thirdFinger1, thirdFinger2);

	Segment fourthFinger2 = Segment(fourthFinger1->endJoint, resizeFinger*glm::vec3(0.5, 2.5, 0), constraint);
	createTree.append_child(fourthFinger1, fourthFinger2);

	//Segment fifthFinger2 = Segment(fifthFinger1->endJoint, resizeFinger*glm::vec3(0.5, 2.0, 0), constraint);
	Segment fifthFinger2 = Segment(fifthFinger1->endJoint, resizeFinger*glm::vec3(1.4, 1.0, 0), constraint);
	createTree.append_child(fifthFinger1, fifthFinger2);

	segmentTree = createTree;
}

void handTreeLarge() {
	float resizeArm = 4.0f;
	float resizeFinger = 4.0f;
	Constraint *constraint = NULL;
	tree<Segment> createTree;

	tree<Segment>::iterator root = createTree.insert(createTree.begin(), Segment(glm::vec3(0, 0, 0), resizeArm * glm::vec3(0, 1, 0), constraint));

	tree<Segment>::iterator firstFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(-1.0, 1.5, 0), constraint));
	tree<Segment>::iterator secondFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(-0.5, 1.5, 0), constraint));
	tree<Segment>::iterator thirdFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(0, 1.5, 0), constraint));
	tree<Segment>::iterator fourthFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(0.5, 1.5, 0), constraint));
	tree<Segment>::iterator fifthFinger1 = createTree.append_child(root, Segment(root->endJoint, resizeFinger*glm::vec3(0.8, 1.0, 0), constraint));

	Segment firstFinger2 = Segment(firstFinger1->endJoint, resizeFinger*glm::vec3(-1.0, 2.5, 0), constraint);
	createTree.append_child(firstFinger1, firstFinger2);

	Segment secondFinger2 = Segment(secondFinger1->endJoint, resizeFinger*glm::vec3(-0.5, 2.5, 0), constraint);
	createTree.append_child(secondFinger1, secondFinger2);

	Segment thirdFinger2 = Segment(thirdFinger1->endJoint, resizeFinger*glm::vec3(0, 2.5, 0), constraint);
	createTree.append_child(thirdFinger1, thirdFinger2);

	Segment fourthFinger2 = Segment(fourthFinger1->endJoint, resizeFinger*glm::vec3(0.5, 2.5, 0), constraint);
	createTree.append_child(fourthFinger1, fourthFinger2);

	Segment fifthFinger2 = Segment(fifthFinger1->endJoint, resizeFinger*glm::vec3(1.4, 1.0, 0), constraint);
	createTree.append_child(fifthFinger1, fifthFinger2);

	segmentTree = createTree;
}

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

void drawFingerSplats(const int program, const float radius, const bool asLight) {

	float resizeFinger = 4.0f;
	float r = 4; //10

	glUseProgram(program);
	uniform(program, "viewMatrix", viewMatrix);
	uniform(program, "projMatrix", projMatrix);

	glm::vec3 splatColors[6] = { glm::vec3(1, 0, 0),
		glm::vec3(1, 1, 0),
		glm::vec3(0, 1, 0),
		glm::vec3(0, 1, 1),
		glm::vec3(0, 0, 1),
		glm::vec3(1, 0, 1) };

	glm::vec3 fingers[5] = {
		resizeFinger*glm::vec3(-1.0, 1.5, 0),
		resizeFinger*glm::vec3(-0.5, 1.5, 0),
		resizeFinger*glm::vec3(0, 1.5, 0),
		resizeFinger*glm::vec3(0.5, 1.5, 0),
		resizeFinger*glm::vec3(1.4, 1.0, 0)
	};

	float clampedAngle;
	float y;
	float z;

	for (int i = 0; i < 4; ++i) {
		//Angle in global variable: rotationAngle;
		clampedAngle = glm::clamp(std::fmod(rotationAngle / 2, (0.8 * M_PI)), -2.0, -0.8);
		std::cout << "clampedAngle: " << clampedAngle << std::endl;
		y = r * cos(clampedAngle);
		z = r * sin(clampedAngle);

		fingers[i] -= glm::vec3(0.0f, y, z);

		uniform(program, "color", splatColors[i]);
		uniform(program, "modelMatrix", glm::translate(fingers[i]) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));

		glm::vec4 homogenTarget = (glm::translate(fingers[i]) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius))) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

		targets[i] = glm::vec3(homogenTarget.x, homogenTarget.y, homogenTarget.z);

		sphere->draw();
	}
	//clampedAngle = glm::clamp(std::fmod(rotationAngle / 2, (0.8 * M_PI)), -2.0, -0.8);
	//y = r * cos(clampedAngle);
	//z = r * sin(clampedAngle);
	//fingers[4] -= glm::vec3(0.0f, y, z);

	//uniform(program, "color", splatColors[4]);
	//uniform(program, "modelMatrix", glm::translate(fingers[4]) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius)));
	//glm::vec4 homogenTarget = (glm::translate(fingers[4]) * glm::scale(glm::mat4(), glm::vec3(radius, radius, radius))) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	//targets[4] = glm::vec3(homogenTarget.x, homogenTarget.y, homogenTarget.z);
	//sphere->draw();

	glUseProgram(0);
}

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

	if (m_currentTarget == TARGET1) {
		drawSplats(gbufferShader, 0.1f, false);
	}
	else if (m_currentTarget == TARGET2) {
		drawFingerSplats(gbufferShader, 0.1f, false);
	}
	if (changeMesh) {
		changeMesh = false;
		if (m_currentMesh == TREE1) {
			
			initiateTestTree();
			fabrik = new FABRIK(segmentTree, targets);
		}
		if (m_currentMesh == TREE2) {

			initiateTestTree2();
			fabrik = new FABRIK(segmentTree, targets);
		}
		if (m_currentMesh == TREE3) {

			initiateTestTree3();
			fabrik = new FABRIK(segmentTree, targets);
		}
		if (m_currentMesh == HAND) {

			handTree();
			fabrik = new FABRIK(segmentTree, targets);
		}
	}

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

		






 


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

//Constants for test-math
float width0 = 0.3f;
float width1 = 0.4f;
float width2 = 0.5f;
float width3 = 0.8f;

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
bool fabrikEllipticConeSolver = false;
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
solidEllipticCone *ellipticCone = 0;
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
glm::vec3 dir = glm::normalize(glm::vec3(12.0f, 8.0f, 4.0f));
//glm::vec3 dir = glm::normalize(glm::vec3(-3.0f, -3.0f, -3.0f));


//FABRIK
FABRIK *fabrik = 0;
//Constraints Value
float angleCone = M_PI / 6;
glm::vec3 planeNormal = glm::vec3(1.0f, 0.0f, 0.0f);
float radiusHinge = 2.0f;
//Constraints
Constraint *constraintCone = new ConeConstraint(M_PI / 6);
//Constraint *constraintEllipticCone = new EllipticConeConstraint(1.0f, 1.0f, 1.0f, 1.0f);
Constraint *constraintEllipticCone = new EllipticConeConstraint(1.0f, 0.5f, 1.5f, 0.4f);
Constraint *constraintPlane = new PlaneConstraint(planeNormal);
Constraint *constraintHinge = new HingeConstraint(radiusHinge);


bool doOnce = true;
bool debugEllipse = false;
bool coordinateSystem = false;

//Model
typedef enum { TREE1, TREE2, TREE3, RANDOM, HAND } MESH_TYPE;
MESH_TYPE m_currentMesh = TREE1;
bool changeMesh = false;
//Target
typedef enum { TARGET1, TARGET2} TARGET_TYPE;
TARGET_TYPE m_currentTarget = TARGET1;
void setupTweakBar() {

	TwInit(TW_OPENGL_CORE, NULL);

	//WEB: http://ogldev.atspace.co.uk/www/tutorial48/tutorial48.html
	TwInit(TW_OPENGL_CORE, NULL);
	tweakBar = TwNewBar("Settings");
	

	// A variable for the current selection - will be updated by ATB
	
	// Array of drop down items
	TwEnumVal Meshes[] = { { TREE1, "Tree1" },{ TREE2, "Tree2" }, { TREE3, "Tree3" },{ RANDOM, "Random" }, { HAND, "Hand" } };
	// ATB identifier for the array
	TwType MeshTwType = TwDefineEnum("MeshType", Meshes, 5);
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
	TwAddVarRW(tweakBar, "Elliptic Cone", TW_TYPE_BOOLCPP, &fabrikEllipticConeSolver, " label='Elliptic Cone' ");
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
	TwAddSeparator(tweakBar, "", NULL);
	TwAddSeparator(tweakBar, "", NULL);
	TwAddVarRW(tweakBar, "Debug Ellipse", TW_TYPE_BOOLCPP, &debugEllipse, " label='Debug Ellipse' ");
	TwAddVarRW(tweakBar, "xyz-Axis", TW_TYPE_BOOLCPP, &coordinateSystem, " label='xyz-Axis' ");
	TwAddVarRW(tweakBar, "Direction", TW_TYPE_DIR3F, &dir, "label='Dirction'");
}

void initiateTestTree() {
	Constraint *constraint = NULL;
	tree<Segment> createTree;
	tree<Segment>::iterator root = createTree.insert(createTree.begin(), Segment(glm::vec3(0, 0, 0), 2.5f, glm::radians(-90.0f), constraint));
	root->update();

	tree<Segment>::iterator first = createTree.append_child(root, Segment(root->endJoint, 2.5f, glm::radians(-90.0f), constraint));
	first->update();

	tree<Segment>::iterator second = createTree.append_child(first, Segment(first->endJoint, 2.5f, glm::radians(-135.0f), constraint));
	second->update();

	tree<Segment>::iterator third = createTree.append_child(first, Segment(first->endJoint, 2.5f, glm::radians(-45.0f), constraint));
	third->update();

	Segment leaf1 = Segment(second->endJoint, 2, glm::radians(-135.0f), constraint);
	leaf1.update();
	createTree.append_child(second, leaf1);

	Segment leaf2 = Segment(second->endJoint, 3, glm::radians(-90.0f), constraint);
	leaf2.update();
	createTree.append_child(second, leaf2);

	Segment leaf3 = Segment(second->endJoint, 2, glm::radians(-45.0f), constraint);
	leaf3.update();
	createTree.append_child(second, leaf3);

	Segment leaf4 = Segment(third->endJoint, 3, glm::radians(-100.0f), constraint);
	leaf4.update();
	createTree.append_child(third, leaf4);

	Segment leaf5 = Segment(third->endJoint, 3, glm::radians(-60.0f), constraint);
	leaf5.update();
	createTree.append_child(third, leaf5);

	segmentTree = createTree;
}

Constraint* getRandomConstraint() {
	int randomNumber = rand() % 3;

	//std::cout << randomNumber << std::endl;

	if (randomNumber == 0) {
		return constraintCone;
	}
	if (randomNumber == 1) {
		return constraintPlane;
	}
	if (randomNumber == 2) {
		return constraintHinge;
	}

	return NULL;
}

void initiateRandomTestTree() {
	Constraint *constraint = NULL;
	tree<Segment> createTree;
	tree<Segment>::iterator root = createTree.insert(createTree.begin(), Segment(glm::vec3(0, 0, 0), 3, glm::radians(-90.0f), getRandomConstraint()));
	root->update();

	tree<Segment>::iterator first = createTree.append_child(root, Segment(root->endJoint, 3, glm::radians(-90.0f), getRandomConstraint()));
	first->update();

	tree<Segment>::iterator second = createTree.append_child(first, Segment(first->endJoint, 3, glm::radians(-135.0f), getRandomConstraint()));
	second->update();

	tree<Segment>::iterator third = createTree.append_child(first, Segment(first->endJoint, 3, glm::radians(-45.0f), getRandomConstraint()));
	third->update();

	Segment leaf1 = Segment(second->endJoint, 2, glm::radians(-135.0f), getRandomConstraint());
	leaf1.update();
	createTree.append_child(second, leaf1);

	Segment leaf2 = Segment(second->endJoint, 4, glm::radians(-90.0f), getRandomConstraint());
	leaf2.update();
	createTree.append_child(second, leaf2);

	Segment leaf3 = Segment(second->endJoint, 2, glm::radians(-45.0f), getRandomConstraint());
	leaf3.update();
	createTree.append_child(second, leaf3);

	Segment leaf4 = Segment(third->endJoint, 4, glm::radians(-100.0f), getRandomConstraint());
	leaf4.update();
	createTree.append_child(third, leaf4);

	Segment leaf5 = Segment(third->endJoint, 4, glm::radians(-60.0f), getRandomConstraint());
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
								 glm::vec3(1, 1, 1),
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
		//std::cout << "clampedAngle: " << clampedAngle << std::endl;
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

	//ellipticCone = new solidEllipticCone(5, 10.0f, 4.0f, 7.0f, 6.0f);
	//ellipticCone = new solidEllipticCone(5, 1.0f, 1.0f, 1.0f, 1.0f);
	ellipticCone = new solidEllipticCone(5, width0, width1, width2, width3);
	ellipticCone->upload();

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

	/*
	Math tests
	*/
	if (debugEllipse) {
		//std::cout << "Math test 3D: " << std::endl;
		glm::vec3 _e0 = glm::vec3(1.0f, 0.0f, 0.0f);
		glm::vec3 _e1 = glm::vec3(0.0f, 1.0f, 0.0f);
		glm::vec3 _e2 = glm::vec3(0.0f, 0.0f, 1.0f);

		//glm::vec3 _start = glm::vec3(1.0f, 1.0f, 2.0f);
		glm::vec3 _start = glm::vec3(0.0f); //Translate vectorstart to (0,  0, 0)

		glm::vec3 _a = glm::vec3(3.0f, 3.0f, 3.0f);
		glm::vec3 _b = _a + 3.0f * glm::normalize(dir);


		//std::cout << "_a: " << _a.x << " " << _a.y << " " << _a.z << " " << std::endl;
		//std::cout << "_b: " << _b.x << " " << _b.y << " " << _b.z << " " << std::endl;

		glm::mat3x3 _rot = (vectorToVectorRotation(_e1, _a));

		glm::vec3 _rotStart = glm::length(_start) * glm::normalize(_rot * _start);

		glm::vec3 _rotA = glm::length(_a) * glm::normalize(_rot * (_a));

		//Point or vector? (Point seems to be correct, why?)
		glm::vec3 _rotB = glm::length(_b) * glm::normalize(_rot * (_b));

		//Worst Case: Both vectors are parallel and point in opposite direction, thus we dont know which part of the ellipse we need to
		//project onto
		//Oppsoite direction dot(a,b) = -1
		//Parallel: cross(a,b) = 0   (floating points, make this nearly impossible. better check for length) 
		//Solution: Small correction into a chosen direction
		float epsilon = 0.3;
		glm::vec3 _rotVecBA = _rotB - _rotA;

		if (glm::dot(_rotA, _rotVecBA) < 0 && glm::cross(_rotA, _rotVecBA).x <= epsilon && glm::cross(_rotA, _rotVecBA).y <= epsilon && glm::cross(_rotA, _rotVecBA).z <= epsilon) {
			_rotB.x += 1.0f;
		}

		glm::vec3 _rotE0 = glm::normalize(_rot * _e0);
		//std::cout << "rotE0: " << _rotE0.x << " " << _rotE0.y << " " << _rotE0.z << std::endl;

		glm::vec3 _rotE1 = glm::normalize(_rot*_e1);
		//std::cout << "rotE1: " << _rotE1.x << " " << _rotE1.y << " " << _rotE1.z << std::endl;

		glm::vec3 _rotE2 = glm::normalize(_rot*_e2);
		//std::cout << "rotE2: " << _rotE2.x << " " << _rotE2.y << " " << _rotE2.z << std::endl;

		//std::cout << "Length: " << glm::length(_b - _a) << std::endl;
		//std::cout << "rot-Length: " << glm::length(_rotB - _rotA) << std::endl;


		/*
		//This is the elliptic math
		*/
		glm::vec3 _coneAxis = (_a - _start);
		glm::vec3 _normalConeAxis = glm::normalize(_coneAxis); //this has to be normalized for the projection

		glm::mat3x3 _rotation = _rot;

		glm::vec3 _unconstraintedVec;// = _b - _a;

		_normalConeAxis = glm::normalize(_rotation * _normalConeAxis);  //this should be glm::vec3(0.0f, 1.0f, 0.0f)

		_unconstraintedVec = glm::length(_b) * glm::normalize(_rotation * _b);
		//_unconstraintedVec = _rotation * _unconstraintedVec;

		//ProjectionVec drawn in yellow
		glm::vec3 projectionVec = glm::dot(_unconstraintedVec, _normalConeAxis) * _normalConeAxis;

		float projectionLength = glm::length(_rotB - projectionVec);

		if (glm::dot(_coneAxis, projectionVec) < 0) {
			projectionVec = -projectionVec;
		}

		//Find in which cone-segment we are
		float currentWidth, currentHeight;
		if (_rotB.x >= 0 && _rotB.z >= 0) {
			currentWidth = width0;
			currentHeight = width1;
		}
		else if (_rotB.x <= 0 && _rotB.z >= 0) {
			currentWidth = width2;
			currentHeight = width1;
		}
		else if (_rotB.x <= 0 && _rotB.z <= 0) {
			currentWidth = width2;
			currentHeight = width3;
		}
		else if (_rotB.x >= 0 && _rotB.z <= 0) {
			currentWidth = width0;
			currentHeight = width3;
		}

		/*std::cout << "currentWidth: " << currentWidth << std::endl;
		std::cout << "currentHeight: " << currentHeight << std::endl;*/
		glm::vec3 constraintProjection;
		glm::vec3 _ellipticB;

		glm::vec3 _nonProjectedRestul;

		if ((_rotB.x*_rotB.x) / (currentWidth*currentWidth) + (_rotB.z*_rotB.z) / (currentHeight*currentHeight) > 1) {
			//Point on ellipse, drawn in teal
			float sigma = atan2(_rotB.z, _rotB.x);
			//std::cout << "sigma: " << sigma << std::endl;
			constraintProjection = glm::vec3(currentWidth * glm::cos(sigma), _rotA.y + 1.0f, currentHeight * glm::sin(sigma));
			_ellipticB = glm::length(_rotB - _rotA) * (constraintProjection - _rotA) + _rotA;
		}
		else {
			_ellipticB = _rotB;
		}

		_nonProjectedRestul = glm::transpose(_rot) * _ellipticB;

		/*
		Math tests
		*/

		//Draw _start Point
		uniform(gbufferShader, "color", glm::vec3(0, 0, 0));
		uniform(gbufferShader, "modelMatrix", glm::translate(_start) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
		sphere->draw();

		//Draw _a
		uniform(gbufferShader, "color", glm::vec3(0, 0, 0));
		uniform(gbufferShader, "modelMatrix", glm::translate(_a) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
		sphere->draw();

		uniform(gbufferShader, "color", glm::vec3(0, 0, 0));

		glm::vec3 segmentAxis = _a - _start;

		glm::mat3x3 r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

		glm::mat4 translate = glm::translate(_start);

		glm::mat4 rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);

		rotation = glm::transpose(rotation);

		uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, glm::length(segmentAxis), 0.1f)));

		cylinder->draw();

		//Draw _b
		uniform(gbufferShader, "color", glm::vec3(0, 0, 0));
		uniform(gbufferShader, "modelMatrix", glm::translate(_b) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
		sphere->draw();

		uniform(gbufferShader, "color", glm::vec3(0, 0, 0));

		segmentAxis = _b - _a;

		r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

		translate = glm::translate(_a);

		rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);

		rotation = glm::transpose(rotation);

		uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, glm::length(segmentAxis), 0.1f)));

		cylinder->draw();

		/*
		Draw elliptic Projections
		*/
		uniform(gbufferShader, "color", glm::vec3(1, 1, 0));
		uniform(gbufferShader, "modelMatrix", glm::translate(projectionVec) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
		sphere->draw();

		uniform(gbufferShader, "color", glm::vec3(0, 1, 1));
		uniform(gbufferShader, "modelMatrix", glm::translate(constraintProjection) * glm::scale(glm::mat4(), glm::vec3(0.03f, 0.03f, 0.03f)));
		sphere->draw();

		uniform(gbufferShader, "color", glm::vec3(0.5f));
		uniform(gbufferShader, "modelMatrix", glm::translate(_ellipticB) * glm::scale(glm::mat4(), glm::vec3(0.02f, 0.02f, 0.02f)));
		sphere->draw();

		segmentAxis = _ellipticB - _rotA;
		r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);
		translate = glm::translate(_rotA);
		rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);
		rotation = glm::transpose(rotation);
		uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, glm::length(segmentAxis), 0.1f)));
		cylinder->draw();



		uniform(gbufferShader, "color", glm::vec3(0.2f, 0.0f, 0.0f));
		uniform(gbufferShader, "modelMatrix", glm::translate(_nonProjectedRestul) * glm::scale(glm::mat4(), glm::vec3(0.02f, 0.02f, 0.02f)));
		sphere->draw();

		segmentAxis = _nonProjectedRestul - _a;
		r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);
		translate = glm::translate(_a);
		rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);
		rotation = glm::transpose(rotation);
		uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, glm::length(segmentAxis), 0.1f)));
		cylinder->draw();

		/*
		Draw rotated vecs
		*/
		//Draw _start Point
		uniform(gbufferShader, "color", glm::vec3(1.0f));
		uniform(gbufferShader, "modelMatrix", glm::translate(_rotStart) * glm::scale(glm::mat4(), glm::vec3(0.06f, 0.06f, 0.06f)));
		sphere->draw();

		uniform(gbufferShader, "color", glm::vec3(1.0f));

		segmentAxis = _rotA - glm::vec3(0.0f);

		r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

		translate = glm::translate(glm::vec3(0.0f));

		rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);

		rotation = glm::transpose(rotation);

		uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, glm::length(segmentAxis), 0.1f)));

		cylinder->draw();

		//Draw _rotA
		uniform(gbufferShader, "color", glm::vec3(1.0f));
		uniform(gbufferShader, "modelMatrix", glm::translate(_rotA) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
		sphere->draw();

		uniform(gbufferShader, "color", glm::vec3(1.0f));

		segmentAxis = _rotA - _rotStart;

		r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

		translate = glm::translate(_rotStart);

		rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);

		rotation = glm::transpose(rotation);

		uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, glm::length(segmentAxis), 0.1f)));

		cylinder->draw();

		//Draw _rotB
		uniform(gbufferShader, "color", glm::vec3(1.0f));
		uniform(gbufferShader, "modelMatrix", glm::translate(_rotB) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
		sphere->draw();

		uniform(gbufferShader, "color", glm::vec3(1.0f));

		segmentAxis = _rotB - _rotA;

		r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

		translate = glm::translate(_rotA);

		rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);

		rotation = glm::transpose(rotation);

		uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, glm::length(segmentAxis), 0.1f)));

		cylinder->draw();

		/*
		Draw elliptic constraints
		*/
		//First ellipse
		uniform(gbufferShader, "color", glm::vec3(1.0f, 0.5f, 0));
		segmentAxis = _a - glm::vec3(0.0f);
		r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);
		translate = glm::translate(_a);

		rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);

		rotation = glm::transpose(rotation);

		uniform(gbufferShader, "modelMatrix", translate*rotation);

		// Turn on wireframe mode
		glPolygonMode(GL_FRONT, GL_LINE);
		glPolygonMode(GL_BACK, GL_LINE);

		// Draw the box
		ellipticCone->draw();

		//Rotated Ellipse
		// Turn off wireframe mode
		glPolygonMode(GL_FRONT, GL_FILL);
		glPolygonMode(GL_BACK, GL_FILL);

		uniform(gbufferShader, "color", glm::vec3(1.0f, 0.5f, 0));
		segmentAxis = _rotA - glm::vec3(0.0f);
		r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);
		translate = glm::translate(_rotA);

		rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
			r[1][0], r[1][1], r[1][2], 0,
			r[2][0], r[2][1], r[2][2], 0,
			0, 0, 0, 1);

		rotation = glm::transpose(rotation);

		uniform(gbufferShader, "modelMatrix", translate*rotation);

		// Turn on wireframe mode
		glPolygonMode(GL_FRONT, GL_LINE);
		glPolygonMode(GL_BACK, GL_LINE);

		// Draw the box
		ellipticCone->draw();

		// Turn off wireframe mode
		glPolygonMode(GL_FRONT, GL_FILL);
		glPolygonMode(GL_BACK, GL_FILL);

		/*
		Draw euclidean system
		*/
		if (coordinateSystem) {
			////Draw Axis x
			float sizeX = 20.0f;
			uniform(gbufferShader, "color", glm::vec3(1, 0, 0));
			uniform(gbufferShader, "modelMatrix", glm::translate(sizeX*_e0) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
			sphere->draw();

			uniform(gbufferShader, "color", glm::vec3(1, 0, 0));

			segmentAxis = _e0 - glm::vec3(0, 0, 0);

			r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

			translate = glm::translate(glm::vec3(0.0f, 0.0f, 0.0f));

			rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
				r[1][0], r[1][1], r[1][2], 0,
				r[2][0], r[2][1], r[2][2], 0,
				0, 0, 0, 1);

			rotation = glm::transpose(rotation);

			uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, sizeX, 0.1f)));

			cylinder->draw();

			//Draw Axis y
			float sizeY = 20.0f;
			uniform(gbufferShader, "color", glm::vec3(0, 1, 0));
			uniform(gbufferShader, "modelMatrix", glm::translate(sizeY*_e1) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
			sphere->draw();

			uniform(gbufferShader, "color", glm::vec3(0, 1, 0));

			segmentAxis = _e1 - glm::vec3(0, 0, 0);

			r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

			translate = glm::translate(glm::vec3(0.0f, 0.0f, 0.0f));

			rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
				r[1][0], r[1][1], r[1][2], 0,
				r[2][0], r[2][1], r[2][2], 0,
				0, 0, 0, 1);

			rotation = glm::transpose(rotation);

			uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, sizeY, 0.1f)));

			cylinder->draw();

			//Draw Axis z
			float sizeZ = 20.0f;
			uniform(gbufferShader, "color", glm::vec3(0, 0, 1));
			uniform(gbufferShader, "modelMatrix", glm::translate(sizeZ*_e2) * glm::scale(glm::mat4(), glm::vec3(0.05f, 0.05f, 0.05f)));
			sphere->draw();

			uniform(gbufferShader, "color", glm::vec3(0, 0, 1));

			segmentAxis = _e2 - glm::vec3(0, 0, 0);

			r = vectorToVectorRotation(glm::vec3(0.0f, 1.0f, 0.0f), segmentAxis);

			translate = glm::translate(glm::vec3(0.0f, 0.0f, 0.0f));

			rotation = glm::mat4(r[0][0], r[0][1], r[0][2], 0,
				r[1][0], r[1][1], r[1][2], 0,
				r[2][0], r[2][1], r[2][2], 0,
				0, 0, 0, 1);

			rotation = glm::transpose(rotation);

			uniform(gbufferShader, "modelMatrix", translate * rotation* glm::scale(glm::mat4(), glm::vec3(0.1f, sizeZ, 0.1f)));

			cylinder->draw();
		}
	}
	else {
		//Current Target
		if (m_currentTarget == TARGET1) {
			drawSplats(gbufferShader, 0.1f, false);
		}
		else if (m_currentTarget == TARGET2) {
			drawFingerSplats(gbufferShader, 0.1f, false);
		}

		//Current Mesh
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
			if (m_currentMesh == RANDOM) {

				initiateRandomTestTree();
				fabrik = new FABRIK(segmentTree, targets);
			}
			if (m_currentMesh == HAND) {

				handTree();
				fabrik = new FABRIK(segmentTree, targets);
			}
		}

		//FABRIK
		if (m_currentMesh == RANDOM) {
			if (updateChain)
				fabrik->updateChainWithConstraints(targets);
			fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
		}
		else {
			if (fabrikSolver) {
				if (updateChain)
					fabrik->updateChain(targets);
				fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
			}
			else if (fabrikConeSolver) {
				fabrik->changeConstraints(constraintCone);
				if (updateChain)
					fabrik->updateChainWithConstraints(targets);
				fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
				fabrik->drawConstraints(gbufferShader, viewMatrix, projMatrix);
				//fabrik->drawEllipticConstraints(gbufferShader, viewMatrix, projMatrix);
			}
			else if (fabrikEllipticConeSolver) {
				fabrik->changeLastConstraints(constraintEllipticCone);
				if (updateChain)
					fabrik->updateChainWithConstraints(targets);
				fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
				fabrik->drawEllipticConstraints(gbufferShader, viewMatrix, projMatrix);
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
			else if (fabrikHingeSolver) {
				((HingeConstraint*)constraintHinge)->setHingeRadius(radiusHinge);
				fabrik->changeConstraints(constraintHinge);

				if (updateChain) {
					//fabrik->updateChainWithConstraints(targets);
					fabrik->updateHingeChain(targets, radiusHinge);
				}
				fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
				fabrik->drawHingeConstraints(gbufferShader, viewMatrix, projMatrix, radiusHinge);
			}
			else {
				fabrik->drawChain(gbufferShader, 0.1f, viewMatrix, projMatrix);
			}
		}

		gl_check_error("after splatting");	
	}

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
	//Random number generator
	srand(time(NULL));

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

		






 


#version 140

uniform mat4 modelMatrix;
uniform mat4 projMatrix;
uniform mat4 viewMatrix;
  
attribute  vec3 vPosition; 
attribute  vec3 vNormal; 

varying vec3 normal;
varying vec4 pos_ec;

void main() {
	mat3 normalMatrix = transpose(inverse(mat3(viewMatrix * modelMatrix)));

	normal = normalMatrix * vNormal.xyz;
	pos_ec = viewMatrix * modelMatrix * vec4(vPosition, 1);
	gl_Position = projMatrix * pos_ec;
}

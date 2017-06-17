#version 140
  
uniform vec3 color;

varying vec3 normal;
varying vec4 pos_ec;

void main() 
{ 
	gl_FragColor = vec4(color, 1);

}

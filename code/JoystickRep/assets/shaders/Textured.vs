#version 420 core

in vec3 position;
in vec2 uvs;

out vec2 pass_uvs;

uniform mat4 projection;

uniform mat4 transformation;

void main(void){
	gl_Position = projection * transformation * vec4(position, 1.0);
	pass_uvs = uvs;
}

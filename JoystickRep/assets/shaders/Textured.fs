#version 450 core

in vec2 pass_uvs;

out vec4 out_Color;

uniform sampler2D textureSampler;

void main(){
	out_Color = texture(textureSampler,pass_uvs);
}

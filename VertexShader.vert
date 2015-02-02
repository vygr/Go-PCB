#version 330 core

layout(location = 0) in vec2 vert_vertex;

uniform vec2 vert_aspect;
uniform vec2 vert_offset;

void main() {
	gl_Position = vec4(vert_vertex.x * vert_aspect.x + vert_offset.x, vert_vertex.y * vert_aspect.y + vert_offset.y, 0.0, 1.0);
}

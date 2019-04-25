#version 430 core

layout (triangles) in;
layout (triangle_strip) out;
layout (max_vertices = 3) out;

in vec3 colorOut[];
out vec3 colorFinal;

void main(void)
{
    int i;

    float d1 = distance(gl_in[0], gl_in[1]);
    float d2 = distance(gl_in[1], gl_in[2]);
    float d3 = distance(gl_in[2], gl_in[3]);

    float distance_max = max(max(d1, d2), d3);

    if (distance_max < 10) {
	colorFinal = colorOut[0];
        gl_Position = gl_in[0].gl_Position;
        EmitVertex();
	colorFinal = colorOut[1];
        gl_Position = gl_in[1].gl_Position;
        EmitVertex();
	colorFinal = colorOut[2];
        gl_Position = gl_in[2].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}

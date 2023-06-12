#version 120

void main()
{
    vec4 viewPos = vec4(gl_Vertex.xyz, 1.0);
    gl_Position = viewPos;
};
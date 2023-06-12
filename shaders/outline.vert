
#version 120

varying vec2 clipPos;
void main()
{
    // Vertex position in main camera Screen space.
    clipPos = gl_Vertex.xy;
    gl_Position = gl_Vertex;
};

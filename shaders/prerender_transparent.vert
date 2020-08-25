#version 120
#extension GL_EXT_gpu_shader4: enable

varying vec3 normal;
varying vec3 position;
varying vec4 color;
varying vec3 wireInterpColor;
varying float depth;

uniform vec4 osg_MaterialDiffuseColor;
uniform vec4 osg_MaterialAmbientColor;

void main()
{
    color = osg_MaterialDiffuseColor;

    normal = normalize((gl_ModelViewMatrix * vec4(gl_Normal.xyz,0)).xyz);
    vec4 inPos = vec4(gl_Vertex.xyz, 1);
    position = (gl_ModelViewMatrix * inPos).xyz;
    depth = -position.z;

    wireInterpColor = vec3(0,0,0);
    wireInterpColor[gl_VertexID%3] = 1.0;

    // Calculate vertex position in clip coordinates
    gl_Position = gl_ModelViewProjectionMatrix * inPos;
}
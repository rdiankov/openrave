#version 120

varying vec3 normal;
varying vec3 position;
varying vec4 color;
uniform vec4 osg_MaterialDiffuseColor;
uniform vec4 osg_MaterialAmbientColor;

void main()
{
    color = osg_MaterialDiffuseColor;
    normal = normalize(gl_Normal);
    vec4 inPos = vec4(gl_Vertex.xyz, 1);
    position = (gl_ModelViewMatrix * inPos).xyz;

    // Calculate vertex position in clip coordinates
    gl_Position = gl_ModelViewProjectionMatrix * inPos;
}
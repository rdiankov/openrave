#version 120

varying vec4 color;
varying vec3 normal;
varying vec3 position;
uniform bool osg_LightEnabled;

uniform vec4 osg_MaterialDiffuseColor;
uniform vec4 osg_MaterialAmbientColor;

void main()
{
    if(!osg_LightEnabled || length(osg_MaterialDiffuseColor.xyz) < 1e-3) {
        color = gl_Color;
    }
    else {
        color = osg_MaterialDiffuseColor;
    }

    normal = normalize((gl_ModelViewMatrix * vec4(gl_Normal.xyz,0)).xyz);
    vec4 inPos = vec4(gl_Vertex.xyz, 1);
    position = (gl_ModelViewMatrix * inPos).xyz;

    // Calculate vertex position in clip coordinates
    gl_Position = gl_ModelViewProjectionMatrix * inPos;
}

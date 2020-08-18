#version 120

varying vec3 normal;
varying vec3 position;
varying vec4 color;

void main()
{
    gl_FragData[0] = color;
    gl_FragData[1] = vec4(normal, 1.0);
    gl_FragData[2] = vec4(gl_FragCoord.z, 0, 0, 1.0);
};
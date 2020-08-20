#version 120
#extension GL_OES_standard_derivatives : enable
#extension GL_ARB_derivative_control : enable
varying vec3 normal;
varying vec3 position;
varying vec4 color;

float linearize_depth(float d, float zNear,float zFar)
{
    float z_n = 2.0 * d - 1.0;
    float z_e = 2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));
    return z_e;
    //return (zNear * zFar / (zFar - zNear))
}

void main()
{
    gl_FragData[0] = vec4(normal,1);
    //gl_FragData[0] = vec4(vec3(0.11*normal.r + 0.59*normal.g + 0.3 * normal.b),1);
    float linearDepth = gl_FragCoord.w;////5 * linearize_depth(gl_FragCoord.z, 0.01, 100);
    gl_FragData[1] = vec4(linearDepth, linearDepth, linearDepth, 1.0);
    
};
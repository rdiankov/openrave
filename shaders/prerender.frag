#version 120
#extension GL_OES_standard_derivatives : enable
#extension GL_ARB_derivative_control : enable
varying vec3 normal;
varying vec3 position;
varying vec4 color;
varying float depth;

float LuminanceFromRgb(vec3 rgb)
{
  return 0.2126*rgb.r + 0.7152*rgb.g + 0.0722*rgb.b;
}

void main()
{
    gl_FragData[0] = vec4(LuminanceFromRgb(normal), depth, 0, 1);
    
};
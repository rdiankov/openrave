#version 120

varying vec3 normal;
varying vec3 position;
varying vec4 color;
varying float depth;
uniform int isSelected;
uniform vec2 textureSize;

float LuminanceFromRgb(vec3 rgb)
{
  return 0.2126*rgb.r + 0.7152*rgb.g + 0.0722*rgb.b;
}

float linearize_depth(float d, float zNear,float zFar)
{
    float z_n = 2.0 * d - 1.0;
    float z_e = 2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));
    return z_e;
}

void main()
{
    vec2 texCoord = gl_FragCoord.xy / textureSize;
    vec3 nNormal = normalize(normal);
    gl_FragData[0] = color;
    gl_FragData[1] = vec4(nNormal, 1);
    gl_FragData[2] = vec4(LuminanceFromRgb(nNormal), depth , isSelected, 1);
    if(isSelected == 1) {
        gl_FragDepth = gl_FragCoord.z / 10;
        return;
    }
    gl_FragDepth = gl_FragCoord.z;
};
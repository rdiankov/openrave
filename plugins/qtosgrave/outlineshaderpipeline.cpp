
#include "osgcartoon.h"
#include "renderutils.h"
#include "outlineshaderpipeline.h"
#include <sstream>

#include <QObject>
#include <QDateTime>
#include <osg/Depth>
#include <osg/CullFace>
#include <osg/BlendFunc>
#include <QFileSystemWatcher>

#include "qtosg.h"

#include <string>

// LOAD_SHADERS_FROM_FOLDER_PATH: if defined, will make pipeline load (and automatically auto refresh when changed) shaders from the given path set by LOAD_SHADERS_FROM_FOLDER_PATH
// this is tipically used during development / debugging. Once shipping, the shader must be embedded in c++ code. See https://github.com/pablocael/shaderfile-to-embed-c-string
// to auto generate c++ strings from shaders.
// if LOAD_SHADERS_FROM_FOLDER_PATH is undefiend, it will not auto reload shaders or read from any folder. Instead, it will use the embedded C++ code.
// < uncomment the below line to load/auto reload shaders from files>
//#define LOAD_SHADERS_FROM_FOLDER_PATH "/home/mujin/mujin/checkoutroot/openrave/shaders"

// To convert glsl files into embedable strings please refer to git@github.com:pablocael/shaderfile-to-embed-c-string.git
///data/shaders/outline.vert
const std::string outline_vert =
	"\n"
	"#version 120\n"
	"\n"
	"varying vec2 clipPos;\n"
	"void main()\n"
	"{\n"
	"    // Vertex position in main camera Screen space.\n"
	"    clipPos = gl_Vertex.xy;\n"
	"    gl_Position = gl_Vertex;\n"
"};\n";

///data/shaders/perpixellighting.vert
const std::string perpixellighting_vert =
	"#version 120\n"
	"\n"
	"varying vec4 color;\n"
	"varying vec3 normal;\n"
	"varying vec3 position;\n"
	"uniform bool osg_LightEnabled;\n"
	"\n"
	"uniform vec4 osg_MaterialDiffuseColor;\n"
	"uniform vec4 osg_MaterialAmbientColor;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    if(!osg_LightEnabled || length(osg_MaterialDiffuseColor.xyz) < 1e-3) {\n"
	"        color = gl_Color;\n"
	"    }\n"
	"    else {\n"
	"        color = osg_MaterialDiffuseColor;\n"
	"    }\n"
	"\n"
	"    normal = normalize((gl_ModelViewMatrix * vec4(gl_Normal.xyz,0)).xyz);\n"
	"    vec4 inPos = vec4(gl_Vertex.xyz, 1);\n"
	"    position = (gl_ModelViewMatrix * inPos).xyz;\n"
	"\n"
	"    // Calculate vertex position in clip coordinates\n"
	"    gl_Position = gl_ModelViewProjectionMatrix * inPos;\n"
	"}\n";

///data/shaders/prerender_transparent.vert
const std::string prerender_transparent_vert =
	"#version 120\n"
	"\n"
	"varying vec3 normal;\n"
	"varying vec3 position;\n"
	"varying float depth;\n"
	"\n"
	"uniform vec4 osg_MaterialDiffuseColor;\n"
	"uniform vec4 osg_MaterialAmbientColor;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    normal = normalize((gl_ModelViewMatrix * vec4(gl_Normal.xyz,0)).xyz);\n"
	"    vec4 inPos = vec4(gl_Vertex.xyz, 1);\n"
	"    position = (gl_ModelViewMatrix * inPos).xyz;\n"
	"    depth = -position.z;\n"
	"\n"
	"    // Calculate vertex position in clip coordinates\n"
	"    gl_Position = gl_ModelViewProjectionMatrix * inPos;\n"
	"}";

///data/shaders/prerender.vert
const std::string prerender_vert =
	"#version 120\n"
	"\n"
	"varying vec3 normal;\n"
	"varying vec3 position;\n"
	"varying vec4 color;\n"
	"varying float depth;\n"
	"\n"
	"uniform vec4 osg_MaterialDiffuseColor;\n"
	"uniform vec4 osg_MaterialAmbientColor;\n"
	"\n"
	"void main()\n"
	"{\n"
	"    color = osg_MaterialDiffuseColor;\n"
	"\n"
	"    normal = normalize((gl_ModelViewMatrix * vec4(gl_Normal.xyz,0)).xyz);\n"
	"    vec4 inPos = vec4(gl_Vertex.xyz, 1);\n"
	"    position = (gl_ModelViewMatrix * inPos).xyz;\n"
	"    depth = -position.z;\n"
	"\n"
	"    // Calculate vertex position in clip coordinates\n"
	"    gl_Position = gl_ModelViewProjectionMatrix * inPos;\n"
	"}";

	//openrave/shaders/outline.frag
	const std::string outline_frag =
	"#version 120\n"
	"\n"
	"uniform vec2 textureSize;\n"
	"uniform vec3 outlineColor;\n"
	"uniform vec3 selectionColor;\n"
	"uniform mat3 sobelOpX;\n"
	"uniform mat3 sobelOpY;\n"
	"varying vec2 clipPos;\n"
	"\n"
	"/* Input textures */\n"
	"\n"
	"uniform sampler2D colorTexture0;\n"
	"\n"
	"const float lightIntensity = 1.0f;\n"
	"const float shininess = 35.0f;    // Specular shininess factor\n"
	"const vec3 lightColor = vec3(1, 1, 1);\n"
	"const vec3 ka = vec3(0.2, 0.2, 0.2);            // Ambient reflectivity\n"
	"const vec3 ks = vec3(0.4, 0.4, 0.4);            // Specular reflectivity\n"
	"/* Utilitary functions and macros */\n"
	"\n"
	"const float PI = 3.14159265359;\n"
	"\n"
	"#define NEW_GAUSSIAN_KERNEL_5x5(variable) \\\n"
	"float variable[25]; \\\n"
	"variable[0] = 0.0125786163522;  variable[1] =  0.0251572327044; variable[2] =  0.0314465408805; variable[3] = 0.0251572327044;  variable[4] = 0.0125786163522; \\\n"
	"variable[5] = 0.0251572327044;  variable[6] =  0.0566037735849; variable[7] =  0.0754716981132; variable[8] = 0.0566037735849;  variable[9] = 0.0251572327044; \\\n"
	"variable[10] = 0.0314465408805; variable[11] = 0.0754716981132; variable[12] = 0.0943396226415; variable[13] = 0.0754716981132; variable[14] = 0.0314465408805; \\\n"
	"variable[15] = 0.0251572327044; variable[16] = 0.0566037735849; variable[17] = 0.0754716981132; variable[18] = 0.0566037735849; variable[19] = 0.0251572327044; \\\n"
	"variable[20] = 0.0125786163522; variable[21] = 0.0251572327044; variable[22] = 0.0314465408805; variable[23] = 0.0251572327044; variable[24] = 0.0125786163522;\n"
	"\n"
	"\n"
	"// Rotates a vector rad radians over the regular cartesian axes\n"
	"vec2 Rotate2D(vec2 v, float rad) {\n"
	"  float s = sin(rad);\n"
	"  float c = cos(rad);\n"
	"  return normalize(mat2(c, s, -s, c) * v);\n"
	"}\n"
	"\n"
	"float ApplyDoubleThreshold(vec2 gradient, float weakThreshold, float strongThreshold) {\n"
	"  float gradientLength = abs(gradient.x) + abs(gradient.y);\n"
	"  if (gradientLength < weakThreshold) return 0.;\n"
	"  if (gradientLength < strongThreshold) return .5;\n"
	"  return 1.;\n"
	"}\n"
	"\n"
	"// Returns a vector with the same length as v but its direction is rounded to the nearest\n"
	"// of 8 cardinal directions of the pixel grid. This function is used by GetSuppressedTextureIntensityGradient\n"
	"// to cancel out edges perpendicular to gradient that are not the strongest ones\n"
	"vec2 Round2DVectorAngleToGrid(vec2 v) {\n"
	"  float len = length(v);\n"
	"  vec2 n = normalize(v);\n"
	"  float maximum = -1.;\n"
	"  float bestAngle = 0;\n"
	"  for (int i = 0; i < 8; i++) {\n"
	"    float theta = (float(i) * PI) / 2.;\n"
	"    vec2 u = Rotate2D(vec2(1., 0.), theta);\n"
	"    float scalarProduct = dot(u, n);\n"
	"    if (scalarProduct > maximum) {\n"
	"      bestAngle = theta;\n"
	"      maximum = scalarProduct;\n"
	"    }\n"
	"  }\n"
	"  return len * Rotate2D(vec2(1., 0.), bestAngle);\n"
	"}\n"
	"\n"
	"float LuminanceFromRgb(vec3 rgb)\n"
	"{\n"
	"  return 0.2126*rgb.r + 0.7152*rgb.g + 0.0722*rgb.b;\n"
	"}\n"
	"\n"
	"// fetches a 3x3 window of pixels centered on coord\n"
	"void Fetch3x3(inout mat3 n, sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)\n"
	"{\n"
	"\tfloat w = 1.0 / textureSize.x;\n"
	"\tfloat h = 1.0 / textureSize.y;\n"
	"\n"
	"  for(int i = 0; i < 3; ++i) {\n"
	"    float row = h*(i-1); // from top to bottom\n"
	"    for(int j = 0; j < 3; ++j) {\n"
	"      float col = w*(j-1);\n"
	"      if(channelIndex == -1) {\n"
	"        n[i][j] = LuminanceFromRgb(texture2D(tex, coord + vec2(col, row)).rgb);\n"
	"      }\n"
	"      else {\n"
	"        n[i][j] = texture2D(tex, coord + vec2(col, row))[channelIndex];\n"
	"      }\n"
	"    }\n"
	"  }\n"
	"}\n"
	"\n"
	"// fetches a 5x5 window of pixels centered on coord\n"
	"void Fetch5x5(inout float n[25], sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)\n"
	"{\n"
	"\tfloat w = 1.0 / textureSize.x;\n"
	"\tfloat h = 1.0 / textureSize.y;\n"
	"\n"
	"  for(int i = 0; i < 5; ++i) {\n"
	"    float row = h*(i-2); // from top to bottom\n"
	"    for(int j = 0; j < 5; ++j) {\n"
	"      float col = w*(j-2);\n"
	"      if(channelIndex == -1) {\n"
	"        n[5*i + j] = LuminanceFromRgb(texture2D(tex, coord + vec2(col, row)).rgb);\n"
	"      }\n"
	"      else{\n"
	"        vec4 color = texture2D(tex, coord + vec2(col, row));\n"
	"        n[5*i + j] = color[channelIndex];\n"
	"      }\n"
	"    }\n"
	"  }\n"
	"}\n"
	"\n"
	"// convolutes two 3x3 kernels\n"
	"float Convolute3x3(mat3 filter, mat3 samples)\n"
	"{\n"
	"  return dot(filter[0], samples[0]) + dot(filter[1], samples[1]) + dot(filter[2], samples[2]);\n"
	"}\n"
	"\n"
	"// convolutes two 5x5 kernels\n"
	"float Convolute5x5(float filter[25], float samples[25])\n"
	"{\n"
	"  float result = 0;\n"
	"  for(int i = 0; i < 5; ++i) {\n"
	"    for(int j = 0; j < 5; ++j) {\n"
	"      result = result + filter[5*i +j] * samples[5*i + j];\n"
	"    }\n"
	"  }\n"
	"  return result;\n"
	"}\n"
	"\n"
	"// fetch 3x3 but applying a 5x5 blur gaussian kernel first\n"
	"void Fetch3x3Blurred(inout mat3 n, sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)\n"
	"{\n"
	"\tfloat w = 1.0 / textureSize.x;\n"
	"\tfloat h = 1.0 / textureSize.y;\n"
	"\n"
	"  NEW_GAUSSIAN_KERNEL_5x5(normal5x5Kernel);\n"
	"\n"
	"  float samples[25];\n"
	"\n"
	"  for(int i = 0; i < 3; ++i) {\n"
	"    float row = h*(i-1); // from top to bottom\n"
	"    for(int j = 0; j < 3; ++j) {\n"
	"      float col = w*(j-1);\n"
	"      Fetch5x5(samples, tex, channelIndex, coord + vec2(col, row), resolution);\n"
	"      n[i][j] = Convolute5x5(normal5x5Kernel, samples);\n"
	"    }\n"
	"  }\n"
	"}\n"
	"\n"
	"float l1norm(vec2 v)\n"
	"{\n"
	"  return abs(v.x) + abs(v.y);\n"
	"}\n"
	"\n"
	"vec2 CalculateIntensityGradient_Fast(sampler2D tex, int channelIndex, vec2 coord, vec2 resolution) {\n"
	"  float w = 1.0 / textureSize.x;\n"
	"\tfloat h = 1.0 / textureSize.y;\n"
	"\n"
	"  float xm = (texture2D(tex, coord + vec2( -w, 0 )))[channelIndex];\n"
	"  float xp = (texture2D(tex, coord + vec2( w, 0 )))[channelIndex];\n"
	"  float ym = (texture2D(tex, coord + vec2( 0.0, -h )))[channelIndex];\n"
	"  float yp = (texture2D(tex, coord + vec2( 0.0, h )))[channelIndex];\n"
	"\n"
	"  float dx = (2*xp - 2*xm);\n"
	"  float dy = (2*yp - 2*ym);\n"
	"\n"
	"  return vec2(abs(dx), abs(dy));\n"
	"}\n"
	"\n"
	"\n"
	"vec2 CalculateIntensityGradient(sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)\n"
	"{\n"
	"  mat3 samples;\n"
	"  Fetch3x3(samples, tex, channelIndex, coord, resolution);\n"
	"\n"
	"  return vec2(Convolute3x3(sobelOpX, samples), Convolute3x3(sobelOpY, samples));\n"
	//"  return CalculateIntensityGradient_Fast(tex, channelIndex, coord, resolution);\n" //< much faster but with poor quality edges for some viewing positions
	"}\n"
	"\n"
	"\n"
	"/*\n"
	"  Get the texture intensity gradient of an image where the angle of the direction is rounded to\n"
	"  one of the 8 cardinal directions and gradients that are not local extrema are zeroed out\n"
	" */\n"
	"vec2 GetSuppressedTextureIntensityGradient(sampler2D tex, int channelIndex, vec2 textureCoord, vec2 resolution) {\n"
	"  float delta = 0;\n"
	"  vec2 gradientOriginal = CalculateIntensityGradient(tex, channelIndex, textureCoord, resolution);\n"
	"  float gradientLength = length(gradientOriginal);\n"
	"  vec2 gradient = Round2DVectorAngleToGrid(gradientOriginal);\n"
	"  vec2 gradientStep = normalize(gradient) / resolution;\n"
	"  vec2 gradientPlusStep = CalculateIntensityGradient(tex, channelIndex, textureCoord + gradientStep, resolution);\n"
	"  if (length(gradientPlusStep)-delta >= gradientLength) return vec2(0.);\n"
	"  vec2 gradientMinusStep = CalculateIntensityGradient(tex, channelIndex, textureCoord - gradientStep, resolution);\n"
	"  if (length(gradientMinusStep)-delta >= gradientLength) return vec2(0.);\n"
	"  return gradient;\n"

	"}\n"
	"\n"
	"float ApplyHysteresis( sampler2D tex, int channelIndex, vec2 textureCoord, vec2 resolution, float weakThreshold, float strongThreshold) {\n"
	"  float dx = 1. / resolution.x;\n"
	"  float dy = 1. / resolution.y;\n"
	"  for (int i = 0; i < 3; i++) {\n"
	"    for (int j = 0; j < 3; j++) {\n"
	"      vec2 ds = vec2(\n"
	"        -dx + (float(i) * dx),\n"
	"        -dy + (float(j) * dy));\n"
	"      vec2 gradient = GetSuppressedTextureIntensityGradient(tex, channelIndex, clamp(textureCoord + ds, vec2(0.), vec2(1.)), resolution);\n"
	"      float edge = ApplyDoubleThreshold(gradient, weakThreshold, strongThreshold);\n"
	"      if (edge == 1.) return 1.;\n"
	"    }\n"
	"  }\n"
	"  return 0.;\n"
	"}\n"
	"\n"
	"\n"
	"float Canny(sampler2D tex, int channelIndex, vec2 coord, vec2 resolution, float weakThreshold, float strongThreshold)\n"
	"{\n"
	"  vec2 gradient = GetSuppressedTextureIntensityGradient(tex, channelIndex, coord, resolution);\n"
	"  float edge = ApplyDoubleThreshold(gradient, weakThreshold, strongThreshold);\n"
	"  if(edge == .5) {\n"
	"    edge = ApplyHysteresis(tex, channelIndex, coord, resolution, weakThreshold, strongThreshold);\n"
	"  }\n"
	"  return edge;\n"
	"}\n"
	"\n"
	"\n"
	"vec3 LightModel(vec3 normal, vec3 diffuseColor)\n"
	"{\n"
	"    // Calculate the vector from the light to the fragment\n"
	"    vec3 s = normalize(vec3(-clipPos.xy,1));\n"
	"\n"
	"    // Calculate the vector from the fragment to the eye position\n"
	"    // (origin since this is in eye or camera space)\n"
	"    vec3 v = s;;\n"
	"\n"
	"    // Reflect the light beam using the normal at this fragment\n"
	"    vec3 r = reflect(-s, normal);\n"
	"\n"
	"    // Calculate the diffuse component\n"
	"    float diffuse = max(dot(s, normal), 0.0);\n"
	"\n"
	"    // Calculate the specular component\n"
	"    float specular = 0.0;\n"
	"    if (dot(s, normal) > 0.0)\n"
	"        specular = (shininess / (8.0 * 3.14)) * pow(max(dot(r, v), 0.0), shininess);\n"
	"\n"
	"    // Lookup diffuse and specular factor\n"
	"    // Combine the ambient, diffuse and specular contributions\n"
	"    return vec3(lightIntensity, lightIntensity, lightIntensity) * ((ka + diffuse) * diffuseColor + specular * ks);\n"
	"}\n"
	"\n"
	"\n"
	"void main()\n"
	"{\n"
	"  vec2 texCoord = gl_FragCoord.xy / textureSize;\n"
	"  float depthValue = texture2D(colorTexture0,texCoord).y;\n"
	"  float adaptativeDepthThreshold = smoothstep(0, depthValue, 1);\n"
	"  vec2 normalThreshold = vec2(0.6, 0.01);\n"
	"  vec2 depthThreshold = vec2(0.05, 0.01);\n"
	"\n"
	"  bool selected = texture2D(colorTexture0, texCoord)[2] > 0.5;\n"
	"  float edgeNormal = Canny(colorTexture0, 0, texCoord, textureSize, normalThreshold.x, normalThreshold.y);\n"
	"  float edgeDepth = Canny(colorTexture0, 1, texCoord, textureSize, depthThreshold.x, depthThreshold.y);\n"
	"\n"
	"  float edge = max(edgeNormal, edgeDepth) * 5 * adaptativeDepthThreshold;\n"
	"\n"
	"  if(selected) {\n"
	"    gl_FragColor = vec4(selectionColor, edge+0.25);\n"
	"    return;\n"
	"  }\n"
	"\n"
	"   gl_FragColor = vec4(outlineColor, edge);\n"
	"};\n"
	"\n"
	"\n";



///data/shaders/perpixellighting.frag
const std::string perpixellighting_frag =
	"\n"
	"#version 120\n"
	"\n"
	"varying vec4 color;\n"
	"varying vec3 normal;\n"
	"varying vec3 position;\n"
	"uniform bool osg_LightEnabled;\n"
	"\n"
	"const float lightIntensity = 1.0f;\n"
	"const float shininess = 40.0f;    // Specular shininess factor\n"
	"const vec3 lightColor = vec3(1, 1, 1);\n"
	"const vec3 ka = vec3(0.2, 0.2, 0.2);            // Ambient reflectivity\n"
	"const vec3 ks = vec3(0.2, 0.2, 0.2);            // Specular reflectivity\n"
	"/* Utilitary functions and macros */\n"
	"\n"
	"const float PI = 3.14159265359;\n"
	"vec3 LightModel(vec3 normal, vec3 position, vec3 diffuseColor)\n"
	"{\n"
	"    // Calculate the vector from the light to the fragment\n"
	"    vec3 s = normalize(-normalize(position));\n"
	"\n"
	"    // Calculate the vector from the fragment to the eye position\n"
	"    // (origin since this is in 'eye' or 'camera' space)\n"
	"    vec3 v = s;;\n"
	"    vec3 nNormal = normalize(normal);\n"
	"\n"
	"    // Reflect the light beam using the normal at this fragment\n"
	"    vec3 r = reflect(-s, nNormal);\n"
	"\n"
	"    // Calculate the diffuse component\n"
	"    float diffuse = max(dot(s, nNormal), 0.0);\n"
	"\n"
	"    // Calculate the specular component\n"
	"    float specular = 0.0;\n"
	"    if (dot(s, nNormal) > 0.0)\n"
	"        specular = (shininess / (8.0 * 3.14)) * pow(max(dot(r, v), 0.0), shininess);\n"
	"\n"
	"    // Lookup diffuse and specular factor\n"
	"    // Combine the ambient, diffuse and specular contributions\n"
	"    return vec3(lightIntensity, lightIntensity, lightIntensity) * ((ka + diffuse) * diffuseColor + specular * ks);\n"
	"}\n"
	"\n"
	"\n"
	"void main()\n"
	"{\n"
	"    if(!osg_LightEnabled) {\n"
	"        gl_FragColor = vec4(color.xyz,1);\n"
	"        return;\n"
	"    }\n"
	"    vec3 lighting = LightModel(normal, position, color.rgb);\n"
	"    gl_FragColor = vec4(vec3(lighting), color.a);\n"
	"};\n"
	"\n";

///data/shaders/prerender.frag
const std::string prerender_frag =
	"#version 120\n"
	"\n"
	"varying vec3 normal;\n"
	"varying vec4 color;\n"
	"varying float depth;\n"
	"varying vec3 position;\n"
	"uniform int isSelected;\n"
	"uniform vec2 textureSize;\n"
	"uniform bool outlineEnabled;\n"
	"\n"
	"float LuminanceFromRgb(vec3 rgb)\n"
	"{\n"
	"  return 0.2126*rgb.r + 0.7152*rgb.g + 0.0722*rgb.b;\n"
	"}\n"
	"\n"
	"float linearize_depth(float d, float zNear,float zFar)\n"
	"{\n"
	"    float z_n = 2.0 * d - 1.0;\n"
	"    float z_e = 2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));\n"
	"    return z_e;\n"
	"}\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec2 texCoord = gl_FragCoord.xy / textureSize;\n"
	"    vec3 nNormal = normalize(normal);\n"
	"    if(!outlineEnabled) {\n"
	"        discard;\n"
	"    }\n"
	"    if(isSelected == 1) {\n"
	"        gl_FragColor = vec4(LuminanceFromRgb(nNormal), depth, isSelected, gl_FragCoord.z / 10);\n"
	"        gl_FragDepth = gl_FragCoord.z / 10;\n"
	"        return;\n"
	"    }\n"
	"    gl_FragColor = vec4(LuminanceFromRgb(nNormal), depth, isSelected, gl_FragCoord.z);\n"
	"    gl_FragDepth = gl_FragCoord.z;\n"
	"};";

///data/shaders/prerender_transparent.frag
const std::string prerender_transparent_frag =
	"#version 120\n"
	"\n"
	"varying vec3 normal;\n"
	"varying float depth;\n"
	"varying vec3 position;\n"
	"uniform int isSelected;\n"
	"uniform vec2 textureSize;\n"
	"uniform bool outlineEnabled;\n"
	"uniform sampler2D depthTexture;\n"
	"\n"
	"\n"
	"float LuminanceFromRgb(vec3 rgb)\n"
	"{\n"
	"  return 0.2126*rgb.r + 0.7152*rgb.g + 0.0722*rgb.b;\n"
	"}\n"
	"\n"
	"float linearize_depth(float d, float zNear,float zFar)\n"
	"{\n"
	"    float z_n = 2.0 * d - 1.0;\n"
	"    float z_e = 2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));\n"
	"    return z_e;\n"
	"    //return (zNear * zFar / (zFar - zNear))\n"
	"}\n"
	"\n"
	"void main()\n"
	"{\n"
	"    vec2 texCoord = gl_FragCoord.xy / textureSize;\n"
	"    vec3 nNormal = normalize(normal);\n"
	"\n"
	"    if(!outlineEnabled) {\n"
	"        discard;\n"
	"    }\n"
	"\n"
	"    // perform the depth test manually using depth texture\n"
	"    float v = texture2D(depthTexture,texCoord).a;\n"
	"    if(v > 0 && v < gl_FragCoord.z) {\n"
	"        discard;\n"
	"    }\n"
	"    gl_FragColor = vec4(LuminanceFromRgb(nNormal), depth, isSelected, 1);\n"
	"    if(isSelected == 1) {\n"
	"        gl_FragDepth = gl_FragCoord.z / 10;\n"
	"        return;\n"
	"    }\n"
	"    gl_FragDepth = gl_FragCoord.z;\n"
	"    \n"
	"};";

class RenderStatePassShaderReloader {

public:
	RenderStatePassShaderReloader(RenderPassState* state)
	{
		_shaderFileWatcher.addPath(QString(state->vertShaderFile.c_str()));
		_shaderFileWatcher.addPath(QString(state->fragShaderFile.c_str()));

		QObject::connect(&_shaderFileWatcher, &QFileSystemWatcher::fileChanged, [=](const QString& file){
            state->ReloadShaders();
			RAVELOG_INFO(str(boost::format("file %s has changed, timestamp is: %s..\n")%file.toStdString()%QDateTime::currentDateTime().toString().toStdString()));
        });
	}

private:
	QFileSystemWatcher _shaderFileWatcher;

};

// state control
namespace {
	const float g_NormalThreshold = 0.8f;
}

RenderPassState::RenderPassState()
{
	state = new osg::StateSet();
	autoReloadShaders = false;
	_shaderReloader = nullptr;
}

void RenderPassState::HandleResize(int width, int height)
{
	if(camera) {
		camera->resize(width, height);
	}

	state->addUniform(new osg::Uniform("textureSize", osg::Vec2f(width, height)));
}

void RenderPassState::SetShaderFiles(const std::string& vertShader, const std::string& fragShader, bool autoReload)
{
	vertShaderFile = vertShader;
	fragShaderFile = fragShader;
	RenderUtils::SetShaderProgramFileOnStateSet(state, vertShader, fragShader);
	autoReloadShaders = autoReload;
	if(autoReload) {
		_shaderReloader = new RenderStatePassShaderReloader(this);
	}
}
void RenderPassState::ReloadShaders()
{
	RenderUtils::SetShaderProgramFileOnStateSet(state, vertShaderFile, fragShaderFile);
}

OutlineShaderPipeline::OutlineShaderPipeline()
{
	_outlineColor = osg::Vec3(0, 0, 0);
	_compatibilityMode = false;
	_compatibilityModeSwitch = new osg::Switch();
	_selectedObjectHighlightIntensity = 1.0f; //< from 0 to +inf, a multiplier on the highligh intensity, 0 means off, 1 means normal (e.g: 2 means double intensity)
	_selectedObjectHighlightColor = osg::Vec3(0, 0.95, 0); //< color of the selected object highlight
}

OutlineShaderPipeline::~OutlineShaderPipeline()
{
	for(RenderPassState* state : _renderPassStates) {
		delete state;
	}
}

void OutlineShaderPipeline::SetCompatibilityModeEnabled(bool value)
{
	_compatibilityMode = value;
	_compatibilityModeSwitch->setSingleChildOn(_compatibilityMode? 1 : 0);
}

RenderPassState* OutlineShaderPipeline::CreateSceneToTexturePass(osg::ref_ptr<osg::Camera> mainSceneCamera,
	osg::ref_ptr<osg::Node> mainSceneRoot, int numColorAttachments, const std::string& vshader, const std::string& fshader, osg::Texture* reusedDepthTexture, bool useMultiSamples, const std::vector<osg::ref_ptr<osg::Texture>>& inheritedColorBuffers)
{
	RenderPassState* renderPassState = new RenderPassState();
#ifdef LOAD_SHADERS_FROM_FOLDER_PATH
	renderPassState->SetShaderFiles(vshader, fshader, true);
#else
	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), vshader, fshader);
#endif
	osg::ref_ptr<osg::Group> firstPassGroup = new osg::Group();
	firstPassGroup->setStateSet(renderPassState->state.get());
	firstPassGroup->addChild(mainSceneRoot);

	// clone main camera settings so to render same scene
	renderPassState->camera = new osg::Camera();

	renderPassState->camera->setRenderingCache(0);
	RenderUtils::FBOData fboData;
	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, _maxFBOBufferWidth, _maxFBOBufferHeight, fboData, reusedDepthTexture, numColorAttachments, useMultiSamples, inheritedColorBuffers);
	renderPassState->colorFboTextures = fboData.colorTextures;
	renderPassState->depthFboTexture = fboData.depthTexture;

	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
	renderPassState->camera->setViewMatrix(osg::Matrix::identity());
	renderPassState->camera->setProjectionMatrix(osg::Matrix::identity());
	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	mainSceneCamera->addChild(renderPassState->camera.get());
	renderPassState->camera->addChild(firstPassGroup.get());
	_SetupOutlineShaderUniforms(renderPassState);
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::CreateTextureToTexturePass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader, bool useMultiSamples, osg::Texture* reusedDepthTexture)
{
	RenderPassState* renderPassState = new RenderPassState();
	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());
	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	std::vector<osg::ref_ptr<osg::Texture>> inputTextures = inputPass->colorFboTextures;
	for(unsigned int i = 0; i < inputTextures.size(); ++i) {
		std::ostringstream bufferNumStr;
		bufferNumStr << "colorTexture";
		bufferNumStr << i;
		renderPassState->state->setTextureAttributeAndModes((int)i, inputTextures[i].get(), osg::StateAttribute::ON);
		renderPassState->state->addUniform(new osg::Uniform(bufferNumStr.str().c_str(), (int)i));
	}

#ifdef LOAD_SHADERS_FROM_FOLDER_PATH
	renderPassState->SetShaderFiles(vshader, fshader, true);
#else
	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), vshader, fshader);
#endif

	RenderUtils::FBOData fboData;
	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, _maxFBOBufferWidth, _maxFBOBufferHeight, fboData, reusedDepthTexture, numColorAttachments, useMultiSamples);
	renderPassState->colorFboTextures = fboData.colorTextures;
	renderPassState->depthFboTexture = fboData.depthTexture;
	_SetupOutlineShaderUniforms(renderPassState);
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::CreateTextureToColorBufferPass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader)
{
	RenderPassState* renderPassState = new RenderPassState();
#ifdef LOAD_SHADERS_FROM_FOLDER_PATH
	renderPassState->SetShaderFiles(vshader, fshader, true);
#else
	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), vshader, fshader);
#endif
	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());
	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
	std::vector<osg::ref_ptr<osg::Texture>> inputTextures = inputPass->colorFboTextures;
	for(unsigned int i = 0; i < inputTextures.size(); ++i) {
		std::ostringstream bufferNumStr;
		bufferNumStr << "colorTexture";
		bufferNumStr << i;
		renderPassState->state->setTextureAttributeAndModes((int)i, inputTextures[i].get(), osg::StateAttribute::ON);
		renderPassState->state->addUniform(new osg::Uniform(bufferNumStr.str().c_str(), (int)i));
	}

	_SetupOutlineShaderUniforms(renderPassState);
	return renderPassState;
}

void OutlineShaderPipeline::_SetupOutlineShaderUniforms(RenderPassState* pass)
{
	pass->state->addUniform(new osg::Uniform("isSelected", 0));
	pass->state->addUniform(new osg::Uniform("outlineEnabled", false));
	pass->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	pass->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	pass->state->addUniform(new osg::Uniform("osg_MaterialDiffuseColor", osg::Vec4f(0,0,0,1)));
	pass->state->addUniform(new osg::Uniform("textureSize", osg::Vec2f(_maxFBOBufferWidth, _maxFBOBufferHeight)));

	osg::Matrix3 sobelY;
	sobelY(0,0) = -1; sobelY(1,0) = 0; sobelY(2,0) = 1;
	sobelY(0,1) = -2; sobelY(1,1) = 0; sobelY(2,1) = 2;
	sobelY(0,2) = -1; sobelY(1,2) = 0; sobelY(2,2) = 1;

	// GLSL matrix are column major matrix, so its transposed from usual notation!
	osg::Matrix3 sobelX;
	sobelX(0,0) = 1; sobelX(1,0) = 2; sobelX(2,0) = 1;
	sobelX(0,1) = 0; sobelX(1,1) = 0; sobelX(2,1) = 0;
	sobelX(0,2) = -1; sobelX(1,2) = -2;sobelX(2,2) = -1;
	pass->state->addUniform(new osg::Uniform("sobelOpX", sobelX));
	pass->state->addUniform(new osg::Uniform("sobelOpY", sobelY));
}

osg::ref_ptr<osg::Group> OutlineShaderPipeline::CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera,
	osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight, bool useMultiSamples)
{
	SetInitialFBOTextureSize(1024, 1024);

    osg::ref_ptr<osg::Texture> depthBuffer = nullptr; // do not use extra depth buffer texture, use color to store depth buffer
	mainSceneCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    std::string preRenderVertShader = prerender_vert;
    std::string preRenderFragShader = prerender_frag;
    std::string preRenderTransparentVertShader = prerender_transparent_vert;
    std::string preRenderTransparentFragShader = prerender_transparent_frag;
    std::string outlineVert = outline_vert;
    std::string outlineFrag = outline_frag;

#ifdef LOAD_SHADERS_FROM_FOLDER_PATH
	preRenderVertShader = LOAD_SHADERS_FROM_FOLDER_PATH"/prerender.vert";
    preRenderFragShader = LOAD_SHADERS_FROM_FOLDER_PATH"/prerender.frag";
    preRenderTransparentVertShader = LOAD_SHADERS_FROM_FOLDER_PATH"/prerender_transparent.vert";
    preRenderTransparentFragShader = LOAD_SHADERS_FROM_FOLDER_PATH"/prerender_transparent.frag";
    outlineVert = LOAD_SHADERS_FROM_FOLDER_PATH"/outline.vert";
    outlineFrag = LOAD_SHADERS_FROM_FOLDER_PATH"/outline.frag";
#endif
	RenderPassState* normalAndDepthMapPass = CreateSceneToTexturePass(mainSceneCamera, mainSceneRoot, 1, preRenderVertShader, preRenderFragShader, depthBuffer.get(), useMultiSamples);
	normalAndDepthMapPass->camera->setCullMask(~qtosgrave::TRANSPARENT_ITEM_MASK);

	RenderPassState* outlinePass = CreateTextureToColorBufferPass(normalAndDepthMapPass, 1, outlineVert, outlineFrag);
	RenderPassState* normalAndDepthMapPassTransparency = CreateSceneToTexturePass(mainSceneCamera, mainSceneRoot, 1, preRenderTransparentVertShader, preRenderTransparentFragShader, depthBuffer.get(), useMultiSamples);
	normalAndDepthMapPassTransparency->camera->setCullMask(qtosgrave::TRANSPARENT_ITEM_MASK);

	RenderPassState* outlinePassTransparency = CreateTextureToColorBufferPass(normalAndDepthMapPassTransparency, 1, outlineVert, outlineFrag);

	// export opaque pipiline resulting depth texture to pre-render shader of transparent objects to perform z test
	normalAndDepthMapPassTransparency->state->setTextureAttributeAndModes(5, normalAndDepthMapPass->colorFboTextures[0].get(), osg::StateAttribute::ON);
	normalAndDepthMapPassTransparency->state->addUniform(new osg::Uniform("depthTexture", 5));
	_renderPassStates.push_back(normalAndDepthMapPass);
	_renderPassStates.push_back(outlinePass);
	_renderPassStates.push_back(normalAndDepthMapPassTransparency);
	_renderPassStates.push_back(outlinePassTransparency);

	osg::ref_ptr<osg::Group> passesGroup = new osg::Group();
	passesGroup->addChild(normalAndDepthMapPass->camera.get());
	passesGroup->addChild(normalAndDepthMapPassTransparency->camera.get());

	osg::Group* grp = new osg::Group();
	RenderUtils::SetShaderProgramOnStateSet(grp->getOrCreateStateSet(), perpixellighting_vert, perpixellighting_frag);
	grp->getOrCreateStateSet()->addUniform(new osg::Uniform("osg_LightEnabled", true));
	grp->addChild(mainSceneRoot);
	passesGroup->addChild(grp);
	passesGroup->addChild(outlinePassTransparency->camera.get());
	passesGroup->addChild(outlinePass->camera.get());

	osg::ref_ptr<qtosgrave::OpenRAVECartoon> toon = new qtosgrave::OpenRAVECartoon();
	toon->addChild(mainSceneRoot);
	_compatibilityModeSwitch->addChild(passesGroup.get());
	_compatibilityModeSwitch->addChild(toon);
	_compatibilityModeSwitch->setSingleChildOn(_compatibilityMode? 1 : 0);

	return _compatibilityModeSwitch.get();
}

void OutlineShaderPipeline::HandleResize(int width, int height)
{
	for(RenderPassState* state : _renderPassStates) {
		state->HandleResize(width, height);
	}
}

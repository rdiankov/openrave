#version 120

uniform vec3 outlineColor;
uniform vec3 selectionColor;
uniform float highlightIntensity;
uniform float depthThreshold;
uniform float normalThreshold;
uniform float depthNormalThreshold;
uniform float depthNormalThresholdScale;
uniform vec2 textureSize;

varying vec2 clipPos;

/* Input textures */

uniform sampler2D colorTexture0;
uniform sampler2D colorTexture1;

/* Utilitary functions and macros */

const float PI = 3.14159265359;

#define NEW_GAUSSIAN_KERNEL_5x5(variable) \
float variable[25]; \
variable[0] = 0.0125786163522;  variable[1] =  0.0251572327044; variable[2] =  0.0314465408805; variable[3] = 0.0251572327044;  variable[4] = 0.0125786163522; \
variable[5] = 0.0251572327044;  variable[6] =  0.0566037735849; variable[7] =  0.0754716981132; variable[8] = 0.0566037735849;  variable[9] = 0.0251572327044; \
variable[10] = 0.0314465408805; variable[11] = 0.0754716981132; variable[12] = 0.0943396226415; variable[13] = 0.0754716981132; variable[14] = 0.0314465408805; \
variable[15] = 0.0251572327044; variable[16] = 0.0566037735849; variable[17] = 0.0754716981132; variable[18] = 0.0566037735849; variable[19] = 0.0251572327044; \
variable[20] = 0.0125786163522; variable[21] = 0.0251572327044; variable[22] = 0.0314465408805; variable[23] = 0.0251572327044; variable[24] = 0.0125786163522;

// GLSL matrix are column major matrix, so its transposed from usual notation!
#define NEW_SOBEL_Y_3x3(variable) \
mat3 variable; \
variable[0][0] = -1; variable[1][0] = 0; variable[2][0] = 1; \
variable[0][1] = -2; variable[1][1] = 0; variable[2][1] = 2; \
variable[0][2] = -1; variable[1][2] = 0; variable[2][2] = 1; \

// GLSL matrix are column major matrix, so its transposed from usual notation!
#define NEW_SOBEL_X_3x3(variable) \
mat3 variable; \
variable[0][0] = 1; variable[1][0] = 2; variable[2][0] = 1; \
variable[0][1] = 0; variable[1][1] = 0; variable[2][1] = 0; \
variable[0][2] = -1; variable[1][2] = -2; variable[2][2] = -1; \

// Rotates a vector 'rad' radians over the regular cartesian axes
vec2 Rotate2D(vec2 v, float rad) {
  float s = sin(rad);
  float c = cos(rad);
  return normalize(mat2(c, s, -s, c) * v);
}

float ApplyDoubleThreshold(vec2 gradient, float weakThreshold, float strongThreshold) {
  float gradientLength = length(gradient);
  if (gradientLength < weakThreshold) return 0.;
  if (gradientLength < strongThreshold) return .5;
  return 1.;
}

// Returns a vector with the same length as v but its direction is rounded to the nearest
// of 8 cardinal directions of the pixel grid. This function is used by GetSuppressedTextureIntensityGradient
// to cancel out edges perpendicular to gradient that are not the strongest ones
vec2 Round2DVectorAngleToGrid(vec2 v) {
  float len = length(v);
  vec2 n = normalize(v);
  float maximum = -1.;
  float bestAngle = 0;
  for (int i = 0; i < 8; i++) {
    float theta = (float(i) * PI) / 4.;
    vec2 u = Rotate2D(vec2(1., 0.), theta);
    float scalarProduct = dot(u, n);
    if (scalarProduct > maximum) {
      bestAngle = theta;
      maximum = scalarProduct;
    }
  }
  return len * Rotate2D(vec2(1., 0.), bestAngle);
}

// fetches a 3x3 window of pixels centered on coord
void Fetch3x3(inout mat3 n, sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  for(int i = 0; i < 3; ++i) {
    float row = h*(i-1); // from top to bottom
    for(int j = 0; j < 3; ++j) {
      float col = w*(j-1);
      n[i][j] = texture2D(tex, coord + vec2(col, row))[channelIndex];
    }
  }
}

// fetches a 5x5 window of pixels centered on coord
void Fetch5x5(inout float n[25], sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  for(int i = 0; i < 5; ++i) {
    float row = h*(i-2); // from top to bottom
    for(int j = 0; j < 5; ++j) {
      float col = w*(j-2);
      n[5*i + j] = texture2D(tex, coord + vec2(col, row))[channelIndex];
    }
  }
}

// convolutes two 3x3 kernels
float Convolute3x3(mat3 filter, mat3 samples)
{
  return dot(filter[0], samples[0]) + dot(filter[1], samples[1]) + dot(filter[2], samples[2]);
}

// convolutes two 5x5 kernels
float Convolute5x5(float filter[25], float samples[25])
{
  float result = 0;
  for(int i = 0; i < 5; ++i) {
    for(int j = 0; j < 5; ++j) {
      result = result + filter[5*i +j] * samples[5*i + j];
    }
  }
  return result;
}

// fetch 3x3 but applying a 5x5 blur gaussian kernel first
void Fetch3x3Blurred(inout mat3 n, sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  NEW_GAUSSIAN_KERNEL_5x5(normal5x5Kernel);

  float samples[25];

  for(int i = 0; i < 3; ++i) {
    float row = h*(i-1); // from top to bottom
    for(int j = 0; j < 3; ++j) {
      float col = w*(j-1);
      Fetch5x5(samples, tex, channelIndex, coord + vec2(col, row), resolution);
      n[i][j] = Convolute5x5(normal5x5Kernel, samples);
    }
  }
}

vec2 CalculateIntensityGradient(sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)
{
  NEW_SOBEL_X_3x3(opX);
  NEW_SOBEL_Y_3x3(opY);

  mat3 samples;
  Fetch3x3Blurred(samples, tex, channelIndex, coord, resolution);

  return vec2(Convolute3x3(opX, samples), Convolute3x3(opY, samples));
}

/*
 * Get the texture intensity gradient of an image
 * where the angle of the direction is rounded to
 * one of the 8 cardinal directions and gradients
 * that are not local extrema are zeroed out
 */
vec2 GetSuppressedTextureIntensityGradient(sampler2D tex, int channelIndex, vec2 textureCoord, vec2 resolution) {
  float delta = 0;
  vec2 gradientOriginal = CalculateIntensityGradient(tex, channelIndex, textureCoord, resolution);
  float gradientLength = length(gradientOriginal);
  vec2 gradient = gradientOriginal;
  vec2 gradientStep = normalize(gradient) / resolution;
  vec2 gradientPlusStep = CalculateIntensityGradient(tex, channelIndex, textureCoord + gradientStep, resolution);
  if (length(gradientPlusStep)-delta >= gradientLength) return vec2(0.);
  vec2 gradientMinusStep = CalculateIntensityGradient(tex, channelIndex, textureCoord - gradientStep, resolution);
  if (length(gradientMinusStep)-delta >= gradientLength) return vec2(0.);
  return gradientOriginal;
}

float ApplyHysteresis( sampler2D tex, int channelIndex, vec2 textureCoord, vec2 resolution, float weakThreshold, float strongThreshold) {
  float dx = 1. / resolution.x;
  float dy = 1. / resolution.y;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      vec2 ds = vec2(
        -dx + (float(i) * dx),
        -dy + (float(j) * dy));
      vec2 gradient = GetSuppressedTextureIntensityGradient(tex, channelIndex, clamp(textureCoord + ds, vec2(0.), vec2(1.)), resolution);
      float edge = ApplyDoubleThreshold(gradient, weakThreshold, strongThreshold);
      if (edge == 1.) return 1.;
    }
  }
  return 0.;
}


float Canny(sampler2D tex, int channelIndex, vec2 coord, vec2 resolution, float weakThreshold, float strongThreshold)
{
  vec2 gradient = GetSuppressedTextureIntensityGradient(colorTexture1, channelIndex, coord, resolution);
  float edge = ApplyDoubleThreshold(gradient, weakThreshold, strongThreshold);
  if(edge == .5) {
    edge = ApplyHysteresis(tex, channelIndex, coord, resolution, weakThreshold, strongThreshold);
  }
  return edge;
}

void main()
{
    vec2 texCoord = gl_FragCoord.xy / textureSize;

    float edge = Canny(colorTexture0, 0, texCoord, textureSize, 0.1, 0.2);
    gl_FragColor = vec4(vec3(edge), 1);
};


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

#define NEW_SOBEL_X_3x3(variable) \
mat3 variable; \
variable[0][0] = -1; variable[1][0] = 0; variable[2][0] = 1; \
variable[0][1] = -2; variable[1][1] = 0; variable[2][1] = 2; \
variable[0][2] = -1; variable[1][2] = 0; variable[2][2] = 1; \


#define NEW_SOBEL_Y_3x3(variable) \
mat3 variable; \
variable[0][0] = 1; variable[1][0] = 2; variable[2][0] = 1; \
variable[0][1] = 0; variable[1][1] = 0; variable[2][1] = 0; \
variable[0][2] = -1; variable[1][2] = -2; variable[2][2] = -1; \

// rotates a vector 'rad' radians over the regular cartesian axes
vec2 Rotate2D(vec2 v, float rad) {
  float s = sin(rad);
  float c = cos(rad);
  return mat2(c, s, -s, c) * v;
}

// Returns a vector with the same length as v but its direction is rounded to the nearest
// of 8 cardinal directions of the pixel grid. This function is used by GetSuppressedTextureIntensityGradient
// to cancel out edges perpendicular to gradient that are not the strongest ones
vec2 Round2DVectorAngleToGrid(vec2 v) {
  float len = length(v);
  vec2 n = normalize(v);
  float maximum = -1.;
  float bestAngle;
  for (int i = 0; i < 8; i++) {
    float theta = (float(i) * 2. * PI) / 8.;
    vec2 u = Rotate2D(vec2(1., 0.), theta);
    float scalarProduct = dot(u, n);
    if (scalarProduct > maximum) {
      bestAngle = theta;
      maximum = scalarProduct;
    }
  }
  return len * Rotate2D(vec2(1., 0.), bestAngle);
}

float LuminanceFromRgb(vec3 rgb)
{
  return 0.2126*rgb.r + 0.7152*rgb.g + 0.0722*rgb.b;
}

// fetches a 3x3 window of pixels centered on coord
void Fetch3x3(inout mat3 n, sampler2D tex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  for(int i = 0; i < 3; ++i) {
    float row = h*(i-1); // from top to bottom
    for(int j = 0; j < 3; ++j) {
      float col = w*(j-1);
      n[i][j] = LuminanceFromRgb(texture2D(tex, coord + vec2(col, row)).rgb);
    }
  }
}


// fetches a 5x5 window of pixels centered on coord
void Fetch5x5(inout float n[25], sampler2D tex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  for(int i = 0; i < 5; ++i) {
    float row = h*(i-2); // from top to bottom
    for(int j = 0; j < 5; ++j) {
      float col = w*(j-2);
      n[5*i + j] = LuminanceFromRgb(texture2D(tex, coord + vec2(col, row)).rgb);
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
void Fetch3x3Blurred(inout mat3 n, sampler2D tex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  NEW_GAUSSIAN_KERNEL_5x5(normal5x5Kernel);

  float samples[25];

  for(int i = 0; i < 3; ++i) {
    float row = h*(i-1); // from top to bottom
    for(int j = 0; j < 3; ++j) {
      float col = w*(j-1);
      Fetch5x5(samples, tex, coord + vec2(col, row), resolution);
      n[i][j] = Convolute5x5(normal5x5Kernel, samples);
    }
  }
}

vec2 CalculateIntensityGradient(sampler2D tex, vec2 coord, vec2 resolution)
{
  NEW_SOBEL_X_3x3(opX);
  NEW_SOBEL_Y_3x3(opY);

  mat3 samples;
  Fetch3x3Blurred(samples, tex, coord, resolution);

  return vec2(Convolute3x3(opX, samples), Convolute3x3(opY, samples));
}

/*
 * Get the texture intensity gradient of an image
 * where the angle of the direction is rounded to
 * one of the 8 cardinal directions and gradients
 * that are not local extrema are zeroed out
 */
vec2 GetSuppressedTextureIntensityGradient(sampler2D tex, vec2 textureCoord, vec2 resolution) {
  vec2 gradient = CalculateIntensityGradient(tex, textureCoord, resolution);
  gradient = Round2DVectorAngleToGrid(gradient);
  vec2 gradientStep = normalize(gradient) / resolution;
  float gradientLength = length(gradient);
  vec2 gradientPlusStep = CalculateIntensityGradient(
    tex, textureCoord + gradientStep, resolution);
  if (length(gradientPlusStep) >= gradientLength) return vec2(0.);
  vec2 gradientMinusStep = CalculateIntensityGradient(
    tex, textureCoord - gradientStep, resolution);
  if (length(gradientMinusStep) >= gradientLength) return vec2(0.);
  return gradient;
}


void Canny(sampler2D tex, vec2 coord, vec2 resolution, int channelIndex, float weakThreshold, float strongThreshold)
{
  vec2 gradient = CalculateIntensityGradient(colorTexture1, coord, resolution);
}

void main()
{
    vec2 texCoord = gl_FragCoord.xy / textureSize;

    vec2 gradient1 = GetSuppressedTextureIntensityGradient(colorTexture0, texCoord, textureSize);

    vec2 gradient2 = GetSuppressedTextureIntensityGradient(colorTexture1, texCoord, textureSize);

    gl_FragColor = vec4(vec3(dot(gradient2, gradient2)), 1);
};


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
uniform sampler2D colorTexture0;
uniform sampler2D colorTexture1;

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

float luminanceFromRgb(vec3 rgb)
{
  return 0.2126*rgb.r + 0.7152*rgb.g + 0.0722*rgb.b;
}

// fetches a 3x3 window of pixels centered on coord
void fetch3x3(inout mat3 n, sampler2D tex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  for(int i = 0; i < 3; ++i) {
    float row = h*(i-1); // from top to bottom
    for(int j = 0; j < 3; ++j) {
      float col = w*(j-1);
      n[i][j] = luminanceFromRgb(texture2D(tex, coord + vec2(col, row)).rgb);
    }
  }
}


// fetches a 5x5 window of pixels centered on coord
void fetch5x5(inout float n[25], sampler2D tex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  for(int i = 0; i < 5; ++i) {
    float row = h*(i-2); // from top to bottom
    for(int j = 0; j < 5; ++j) {
      float col = w*(j-2);
      n[5*i + j] = luminanceFromRgb(texture2D(tex, coord + vec2(col, row)).rgb);
    }
  }
}

// convolutes two 3x3 kernels
float convolute3x3(mat3 filter, mat3 samples)
{
  return dot(filter[0], samples[0]) + dot(filter[1], samples[1]) + dot(filter[2], samples[2]);
}

// convolutes two 5x5 kernels
float convolute5x5(float filter[25], float samples[25])
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
void fetch3x3Blurred(inout mat3 n, sampler2D tex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  NEW_GAUSSIAN_KERNEL_5x5(normal5x5Kernel);

  float samples[25];

  for(int i = 0; i < 3; ++i) {
    float row = h*(i-1); // from top to bottom
    for(int j = 0; j < 3; ++j) {
      float col = w*(j-1);
      fetch5x5(samples, tex, coord + vec2(col, row), resolution);
      n[i][j] = convolute5x5(normal5x5Kernel, samples);
    }
  }
}

void sobel(sampler2D tex, vec2 coord, vec2 resolution, inout float normalGradientMagnitude, inout float normalGradientAngle, inout vec2 gradient)
{
  NEW_SOBEL_X_3x3(opX);
  NEW_SOBEL_Y_3x3(opY);

  mat3 samples;
  fetch3x3Blurred(samples, tex, coord, resolution);

  gradient[0] = convolute3x3(opX, samples);
  gradient[1] = convolute3x3(opY, samples);

  normalGradientMagnitude = sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1]);
  normalGradientAngle = atan(gradient[1]/gradient[0]);
}

void canny(sampler2D tex, vec2 coord, vec2 resolution, int channelIndex, float weakThreshold, float strongThreshold)
{
  float depthGradientMagnitude[3];
  float depthGradientAngle[3];

  vec2 gradient;
  sobel(colorTexture1, coord, resolution, depthGradientMagnitude[0], depthGradientAngle[0], gradient);
}

void main()
{
    vec2 texCoord = gl_FragCoord.xy / textureSize;

    vec2 gradient;
    float normalGradientMagnitude;
    float normalGradientAngle;
    sobel(colorTexture0, texCoord, textureSize, normalGradientMagnitude, normalGradientAngle, gradient);

    float normalSobel = normalGradientMagnitude;

    float depthGradientMagnitude;
    float depthGradientAngle;
    sobel(colorTexture1, texCoord, textureSize, depthGradientMagnitude, depthGradientAngle, gradient);

    float depthSobel = depthGradientMagnitude;
    gl_FragColor = vec4(vec3(max(normalSobel,depthSobel)), 1);
};


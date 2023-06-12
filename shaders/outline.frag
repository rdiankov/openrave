#version 120

uniform vec2 textureSize;
uniform vec3 outlineColor;
uniform vec3 selectionColor;
uniform mat3 sobelOpX;
uniform mat3 sobelOpY;
varying vec2 clipPos;

/* Input textures */

uniform sampler2D colorTexture0;

const float lightIntensity = 1.0f;
const float shininess = 35.0f;    // Specular shininess factor
const vec3 lightColor = vec3(1, 1, 1);
const vec3 ka = vec3(0.2, 0.2, 0.2);            // Ambient reflectivity
const vec3 ks = vec3(0.4, 0.4, 0.4);            // Specular reflectivity
/* Utilitary functions and macros */

const float PI = 3.14159265359;

#define NEW_GAUSSIAN_KERNEL_5x5(variable) \
float variable[25]; \
variable[0] = 0.0125786163522;  variable[1] =  0.0251572327044; variable[2] =  0.0314465408805; variable[3] = 0.0251572327044;  variable[4] = 0.0125786163522; \
variable[5] = 0.0251572327044;  variable[6] =  0.0566037735849; variable[7] =  0.0754716981132; variable[8] = 0.0566037735849;  variable[9] = 0.0251572327044; \
variable[10] = 0.0314465408805; variable[11] = 0.0754716981132; variable[12] = 0.0943396226415; variable[13] = 0.0754716981132; variable[14] = 0.0314465408805; \
variable[15] = 0.0251572327044; variable[16] = 0.0566037735849; variable[17] = 0.0754716981132; variable[18] = 0.0566037735849; variable[19] = 0.0251572327044; \
variable[20] = 0.0125786163522; variable[21] = 0.0251572327044; variable[22] = 0.0314465408805; variable[23] = 0.0251572327044; variable[24] = 0.0125786163522;


// Rotates a vector rad radians over the regular cartesian axes
vec2 Rotate2D(vec2 v, float rad) {
  float s = sin(rad);
  float c = cos(rad);
  return normalize(mat2(c, s, -s, c) * v);
}

float ApplyDoubleThreshold(vec2 gradient, float weakThreshold, float strongThreshold) {
  float gradientLength = abs(gradient.x) + abs(gradient.y);
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
    float theta = (float(i) * PI) / 2.;
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
void Fetch3x3(inout mat3 n, sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)
{
  float w = 1.0 / textureSize.x;
  float h = 1.0 / textureSize.y;

  for(int i = 0; i < 3; ++i) {
    float row = h*(i-1); // from top to bottom
    for(int j = 0; j < 3; ++j) {
      float col = w*(j-1);
      if(channelIndex == -1) {
        n[i][j] = LuminanceFromRgb(texture2D(tex, coord + vec2(col, row)).rgb);
      }
      else {
        n[i][j] = texture2D(tex, coord + vec2(col, row))[channelIndex];
      }
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
      if(channelIndex == -1) {
        n[5*i + j] = LuminanceFromRgb(texture2D(tex, coord + vec2(col, row)).rgb);
      }
      else{
        vec4 color = texture2D(tex, coord + vec2(col, row));
        n[5*i + j] = color[channelIndex];
      }
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

float l1norm(vec2 v)
{
  return abs(v.x) + abs(v.y);
}

vec2 CalculateIntensityGradient_Fast(sampler2D tex, int channelIndex, vec2 coord, vec2 resolution) {
  float w = 1.0 / textureSize.x;
  float h = 1.0 / textureSize.y;

  float xm = (texture2D(tex, coord + vec2( -w, 0 )))[channelIndex];
  float xp = (texture2D(tex, coord + vec2( w, 0 )))[channelIndex];
  float ym = (texture2D(tex, coord + vec2( 0.0, -h )))[channelIndex];
  float yp = (texture2D(tex, coord + vec2( 0.0, h )))[channelIndex];

  float dx = (2*xp - 2*xm);
  float dy = (2*yp - 2*ym);

  return vec2(abs(dx), abs(dy));
}


vec2 CalculateIntensityGradient(sampler2D tex, int channelIndex, vec2 coord, vec2 resolution)
{
  mat3 samples;
  Fetch3x3(samples, tex, channelIndex, coord, resolution);

  return vec2(Convolute3x3(sobelOpX, samples), Convolute3x3(sobelOpY, samples));
	//  return CalculateIntensityGradient_Fast(tex, channelIndex, coord, resolution); //< much faster but with poor quality edges for some viewing positions
}


/*
  Get the texture intensity gradient of an image where the angle of the direction is rounded to
  one of the 8 cardinal directions and gradients that are not local extrema are zeroed out
 */
vec2 GetSuppressedTextureIntensityGradient(sampler2D tex, int channelIndex, vec2 textureCoord, vec2 resolution) {
  float delta = 0;
  vec2 gradientOriginal = CalculateIntensityGradient(tex, channelIndex, textureCoord, resolution);
  float gradientLength = length(gradientOriginal);
  vec2 gradient = Round2DVectorAngleToGrid(gradientOriginal);
  vec2 gradientStep = normalize(gradient) / resolution;
  vec2 gradientPlusStep = CalculateIntensityGradient(tex, channelIndex, textureCoord + gradientStep, resolution);
  if (length(gradientPlusStep)-delta >= gradientLength) return vec2(0.);
  vec2 gradientMinusStep = CalculateIntensityGradient(tex, channelIndex, textureCoord - gradientStep, resolution);
  if (length(gradientMinusStep)-delta >= gradientLength) return vec2(0.);
  return gradient;

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
  vec2 gradient = GetSuppressedTextureIntensityGradient(tex, channelIndex, coord, resolution);
  float edge = ApplyDoubleThreshold(gradient, weakThreshold, strongThreshold);
  if(edge == .5) {
    edge = ApplyHysteresis(tex, channelIndex, coord, resolution, weakThreshold, strongThreshold);
  }
  return edge;
}


vec3 LightModel(vec3 normal, vec3 diffuseColor)
{
    // Calculate the vector from the light to the fragment
    vec3 s = normalize(vec3(-clipPos.xy,1));

    // Calculate the vector from the fragment to the eye position
    // (origin since this is in eye or camera space)
    vec3 v = s;;

    // Reflect the light beam using the normal at this fragment
    vec3 r = reflect(-s, normal);

    // Calculate the diffuse component
    float diffuse = max(dot(s, normal), 0.0);

    // Calculate the specular component
    float specular = 0.0;
    if (dot(s, normal) > 0.0)
        specular = (shininess / (8.0 * 3.14)) * pow(max(dot(r, v), 0.0), shininess);

    // Lookup diffuse and specular factor
    // Combine the ambient, diffuse and specular contributions
    return vec3(lightIntensity, lightIntensity, lightIntensity) * ((ka + diffuse) * diffuseColor + specular * ks);
}


void main()
{
  vec2 texCoord = gl_FragCoord.xy / textureSize;
  float depthValue = texture2D(colorTexture0,texCoord).y;
  float adaptativeDepthThreshold = smoothstep(0, depthValue, 1);
  vec2 normalThreshold = vec2(0.6, 0.01);
  vec2 depthThreshold = vec2(0.05, 0.01);

  bool selected = texture2D(colorTexture0, texCoord)[2] > 0.5;
  float edgeNormal = Canny(colorTexture0, 0, texCoord, textureSize, normalThreshold.x, normalThreshold.y);
  float edgeDepth = Canny(colorTexture0, 1, texCoord, textureSize, depthThreshold.x, depthThreshold.y);

  float edge = max(edgeNormal, edgeDepth) * 5 * adaptativeDepthThreshold;

  if(selected) {
    gl_FragColor = vec4(selectionColor, edge+0.25);
    return;
  }

   gl_FragColor = vec4(outlineColor, edge);
};
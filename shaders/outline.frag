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

// fetches a 3x3 window of pixels centered on coord
void fetch3x3(inout vec3 n[9], sampler2D tex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  for(int i = 0; i < 3; ++i) {
    float row = h*(i-1); // from top to bottom
    for(int j = 0; j < 3; ++j) {
      float col = w*(j-1);
      n[3*i + j] = texture2D(tex, coord + vec2(col, row)).xyz;
    }
  }
}

// fetches a 5x5 window of pixels centered on coord
void fetch5x5(inout vec4 n[25], sampler2D tex, vec2 coord, vec2 resolution)
{
	float w = 1.0 / textureSize.x;
	float h = 1.0 / textureSize.y;

  for(int i = 0; i < 5; ++i) {
    float row = h*(i-2); // from top to bottom
    for(int j = 0; j < 5; ++j) {
      float col = w*(j-2);
      n[5*i + j] = texture2D(tex, coord + vec2(col, row));
    }
  }
}

// convolutes two 5x5 kernels
float convolute5x5(float filter[25], vec4 samples[25], int channelIndex) 
{
  float result = 0;
  for(int i = 0; i < 5; ++i) {
    for(int j = 0; j < 5; ++j) {
      result = result + filter[5*i +j] * samples[5*i + j][channelIndex];
    }
  }
  return result;
}

vec4 accessTexel(sampler2D tex, vec2 tc) {
    return texture2D(tex, tc);
}

float sobelIntensity3x3(in vec4 n[9], int componentIndex) {
    float sobel_edge_h = n[2][componentIndex] + (2.0*n[5][componentIndex]) + n[8][componentIndex] - (n[0][componentIndex] + (2.0*n[3][componentIndex]) + n[6][componentIndex]);
    float sobel_edge_v = n[0][componentIndex] + (2.0*n[1][componentIndex]) + n[2][componentIndex] - (n[6][componentIndex] + (2.0*n[7][componentIndex]) + n[8][componentIndex]);
    float sobel = abs(sobel_edge_h) + abs(sobel_edge_v);
    return sobel;
}

void main()
{
    vec2 texCoord = gl_FragCoord.xy / textureSize;
    
    // vec4 depthSamples[9];
    // getNeighbors(colorTexture1, depthSamples, texCoord);
    // float depthValue = depthSamples[4].r;
    // vec3 normal = normalize(vec3(normalSamples[4].xyz));
    // float viewDot = 1 - dot(normalize(normal), normalize(vec3(-clipPos.xy,1)));
    // float normalThreshold01 = clamp((viewDot - depthNormalThreshold) / (1 - depthNormalThreshold), 0.0, 1.0);
    // float normalThresholdDynamic = normalThreshold01 * depthNormalThresholdScale + 1;
    // float eNormal = sobelIntensity(normalSamples, 0);
    // eNormal = eNormal > normalThreshold ? 1.0 : 0.0;
    // float edgeDepth = 20 * sobelIntensity(depthSamples,0);
    // float depthThreshold = depthThreshold * depthValue * normalThresholdDynamic;
    // edgeDepth = edgeDepth > depthThreshold ? 1.0 : 0.0;
    // float edge = max(edgeDepth, eNormal);
    
    // gl_FragColor = vec4(depthValue, depthValue, depthValue, 1.0);
    // gl_FragColor = vec4(edge, edge, edge, 1);
    // gl_FragColor = vec4(0, 0, 0, edgeDepth);
    // gl_FragColor = vec4(0,0,0, eNormal);
    // gl_FragColor = vec4(0, 0, 0, edgeDepth);
    // gl_FragColor = vec4(edgeDepth, edgeDepth, edgeDepth, 1);
    // gl_FragColor = vec4(accessTexel(colorTexture0, texCoord));
    // gl_FragColor = vec4(0, 0, 0, edge);
    //gl_FragColor = vec4(normalSamples[4].xyz, 1.0);

    NEW_GAUSSIAN_KERNEL_5x5(normal5x5Kernel);

    vec4 normalSamples[25];
    fetch5x5(normalSamples, colorTexture0, texCoord, textureSize);

    float v = convolute5x5(normal5x5Kernel, normalSamples, 2);
    //gl_FragColor = accessTexel(colorTexture0, texCoord);
    gl_FragColor = vec4(vec3(v),1);

    //float edge = cannyEdgeDetection(colorTexture1, texCoord, textureSize, 0.05, 0.05);
    //gl_FragColor = mix( vec4(edge, edge, edge, 1), vec4(texture2D(colorTexture1, texCoord)), 1.);
    //gl_FragColor = mix( vec4(edge, edge, edge, 1), vec4(texture2D(colorTexture0, texCoord)), 1);
};


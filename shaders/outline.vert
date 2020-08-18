#version 120

varying vec2 clipPos;
uniform vec3 outlineColor;
uniform vec3 selectionColor;
uniform float highlightIntensity;
uniform float depthThreshold;
uniform float normalThreshold;
uniform float depthNormalThreshold;
uniform float depthNormalThresholdScale;
uniform vec2 textureSize;

uniform sampler2D colorTexture0;
uniform sampler2D colorTexture1;
uniform sampler2D colorTexture2;

vec4 accessTexel(sampler2D tex, vec2 tc) {
    return texture2D(tex, tc);
}
void getNeighbors(sampler2D tex, inout vec4 n[9], vec2 coord)
{
    // n values are stored from - to +, first x then y
    float lineSize = 1.0;
    float w = lineSize / textureSize.x;
    float h = lineSize / textureSize.y;

    n[0] = accessTexel(tex, coord + vec2( -w, -h));
    n[1] = accessTexel(tex, coord + vec2(0.0, -h));
    n[2] = accessTexel(tex, coord + vec2(  w, -h));
    n[3] = accessTexel(tex, coord + vec2( -w, 0.0));
    n[4] = accessTexel(tex, coord);
    n[5] = accessTexel(tex, coord + vec2(  w, 0.0));
    n[6] = accessTexel(tex, coord + vec2( -w, h));
    n[7] = accessTexel(tex, coord + vec2(0.0, h));
    n[8] = accessTexel(tex, coord + vec2(  w, h));
}

float sobelDepthIntensity(in vec4 n[9]) {
    float sobel_edge_h = n[2].r + (2.0*n[5].r) + n[8].r - (n[0].r + (2.0*n[3].r) + n[6].r);
    float sobel_edge_v = n[0].r + (2.0*n[1].r) + n[2].r - (n[6].r + (2.0*n[7].r) + n[8].r);
    float sobel = sqrt((sobel_edge_h * sobel_edge_h) + (sobel_edge_v * sobel_edge_v));
    return sobel;
}

float sobelNormalIntensity(in vec4 n[9]) {
    vec3 sobel_edge_h = n[2].rgb + (2.0*n[5].rgb) + n[8].rgb - (n[0].rgb + (2.0*n[3].rgb) + n[6].rgb);
    vec3 sobel_edge_v = n[0].rgb + (2.0*n[1].rgb) + n[2].rgb - (n[6].rgb + (2.0*n[7].rgb) + n[8].rgb);
    vec3 sobel = sqrt((sobel_edge_h * sobel_edge_h) + (sobel_edge_v * sobel_edge_v));
    return length(sobel.rgb);
}

float edgeNormal(in vec4 n[9]) {

    vec3 n0 = n[0].rgb;
    vec3 n1 = n[1].rgb;
    vec3 n2 = n[2].rgb;
    vec3 n3 = n[3].rgb;

    vec3 d0 = (n1 - n0);
    vec3 d1 = (n3 - n2);

    float edge = sqrt(dot(d0, d0) + dot(d1, d1));
    return edge;
}

void main()
    {
    vec2 texCoord = gl_FragCoord.xy / textureSize;
    vec4 normalSamples[9];
    getNeighbors(colorTexture1, normalSamples, texCoord);
    vec4 depthSamples[9];
    getNeighbors(colorTexture2, depthSamples, texCoord);
    float depthValue = depthSamples[4].r;
    vec3 normal = normalize(vec3(normalSamples[4].xyz));
    float viewDot = 1 - dot(normalize(normal), normalize(vec3(-clipPos.xy,1)));
    float normalThreshold01 = clamp((viewDot - depthNormalThreshold) / (1 - depthNormalThreshold), 0.0, 1.0);
    float normalThresholdDynamic = normalThreshold01 * depthNormalThresholdScale + 1;
    float eNormal = sobelNormalIntensity(normalSamples);
    eNormal = eNormal > normalThreshold ? 1.0 : 0.0;
    float edgeDepth = 20 * sobelDepthIntensity(depthSamples);
    float depthThreshold = depthThreshold * depthValue * normalThresholdDynamic;
    edgeDepth = edgeDepth > depthThreshold ? 1.0 : 0.0;
    float edge = max(edgeDepth, eNormal);
    vec4 originalColor = accessTexel(colorTexture0, texCoord);
    gl_FragColor = vec4(accessTexel(colorTexture1, texCoord).xyz, 1.0);
    gl_FragColor = vec4(mix(originalColor.xyz, vec3(0, 0, 0), edge * depthValue), 1.0);
//    gl_FragColor = vec4(eNormal, eNormal, eNormal, 1.0);
};


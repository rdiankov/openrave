#version 120

uniform vec2 textureSize;
uniform vec2 offSet;
varying vec2 clipPos;

uniform sampler2D colorTexture0;

vec4 accessTexel(sampler2D tex, vec2 tc) {
    return texture2D(tex, tc);
}

void main()
    {
    vec2 texCoord = clipPos - offSet;
    gl_FragColor = accessTexel(colorTexture0, texCoord);
};


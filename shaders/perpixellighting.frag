
#version 120

varying vec4 color;
varying vec3 normal;
varying vec3 position;
uniform bool osg_LightEnabled;

const float lightIntensity = 1.0f;
const float shininess = 40.0f;    // Specular shininess factor
const vec3 lightColor = vec3(1, 1, 1);
const vec3 ka = vec3(0.2, 0.2, 0.2);            // Ambient reflectivity
const vec3 ks = vec3(0.2, 0.2, 0.2);            // Specular reflectivity
/* Utilitary functions and macros */

const float PI = 3.14159265359;
vec3 LightModel(vec3 normal, vec3 position, vec3 diffuseColor)
{
    // Calculate the vector from the light to the fragment
    vec3 s = normalize(-normalize(position));

    // Calculate the vector from the fragment to the eye position
    // (origin since this is in "eye" or "camera" space)
    vec3 v = s;;
    vec3 nNormal = normalize(normal);

    // Reflect the light beam using the normal at this fragment
    vec3 r = reflect(-s, nNormal);

    // Calculate the diffuse component
    float diffuse = max(dot(s, nNormal), 0.0);

    // Calculate the specular component
    float specular = 0.0;
    if (dot(s, nNormal) > 0.0)
        specular = (shininess / (8.0 * 3.14)) * pow(max(dot(r, v), 0.0), shininess);

    // Lookup diffuse and specular factor
    // Combine the ambient, diffuse and specular contributions
    return vec3(lightIntensity, lightIntensity, lightIntensity) * ((ka + diffuse) * diffuseColor + specular * ks);
}


void main()
{
    if(!osg_LightEnabled) {
        gl_FragColor = vec4(color.xyz,1);
        return;
    }
    vec3 lighting = LightModel(normal, position, color.rgb);
    gl_FragColor = vec4(vec3(lighting), color.a);
};


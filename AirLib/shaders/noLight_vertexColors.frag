// simple fragment shader
#version 120

// 'time' contains seconds since the program was linked.
uniform float time;

varying vec3 P;
varying vec3 Normal;

uniform struct LightInfo {
	vec4 position;
	vec3 intensity;
} light;

struct MaterialInfo
{
	vec3 ka;
	vec3 kd;
	vec3 ks;
	float shininess;
	float opacity;
};

uniform MaterialInfo material;

void main()
{
    vec3 a = gl_Color.xyz;
    gl_FragColor = vec4( a.xyz, material.opacity);
}

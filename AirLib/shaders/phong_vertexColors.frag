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
	vec3 N = normalize(Normal);
    vec3 L = normalize(vec3(light.position));
	vec3 col;
	col.x = gl_Color.x; col.y = gl_Color.y; col.z = gl_Color.z; 
    vec3 a = light.intensity * col * 0.1;
	float sDotN = dot(L,N);
    vec3 d = max(0.0, sDotN) * light.intensity * col * 1;
	vec3 V = normalize(-P);
    vec3 R = reflect(-L,N);
    vec3 s = step(0.0, sDotN) * light.intensity * col * 0.1 * pow(max(0.0, dot(R,V)), 1.0);
		
    gl_FragColor = vec4((a+d+s).xyz, material.opacity);
}

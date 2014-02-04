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
};

uniform MaterialInfo material;


void main()
{
	vec3 lightIntensity = light.intensity;

	vec3 N = normalize(Normal);
    vec3 L = normalize(vec3(light.position.xyz));
    vec3 a = light.intensity * material.ka;
	float sDotN = dot(L,N);
    vec3 d = max(0.0, sDotN) * lightIntensity * material.kd ;
	vec3 V = normalize(-P);
    vec3 R = reflect(-L,N);
    vec3 s = step(0.0, sDotN) * lightIntensity * material.ks * pow(max(0.0, dot(R,V)), material.shininess);

	float alpha = 0.0;
	float k = smoothstep(alpha, 1.0, 1.0 - max(0.0, dot(V,N)));

    gl_FragColor = vec4((k*(a+d)).xyz, 1.0);
}

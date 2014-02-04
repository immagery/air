// simple vertex shader
#version 120

varying vec3 P;
varying vec3 Normal;

void main()
{
    P              = vec3(gl_ModelViewMatrix  * gl_Vertex);
	Normal         = mat3(gl_ModelViewMatrix) * gl_Normal;
	gl_Position    = gl_ModelViewProjectionMatrix * gl_Vertex;
	gl_FrontColor  = gl_Color;
	gl_TexCoord[0] = gl_MultiTexCoord0;
}

#version 410

layout (location=0) in vec3 VertexPosition;
layout (location=1) in vec2 VTexCoord;
layout (location=2) in vec3 VertexNormal;
layout (location=3) in vec4 VertexColor;

out vec3 Position;
out vec3 Normal;
out vec2 TexCoord;
out vec4 Color;


uniform mat4 ModelViewMatrix;
uniform mat3 NormalMatrix;
uniform mat4 ProjectionMatrix;
uniform mat4 MVP;

uniform float PointSize;

// ---------------------------------------------------------
void main()
{
    Color = VertexColor;

    Normal = normalize( NormalMatrix * VertexNormal);
    Position = vec3( ModelViewMatrix * vec4(VertexPosition,1.0) );
    TexCoord = VTexCoord;
    /* TexCoord = vec2(-0.1,-0.1); */
    /* TexCoord.x = VTexCoord.y; */
    /* TexCoord.y = VTexCoord.x; */

    gl_Position = MVP * vec4(VertexPosition,1.0);
    gl_PointSize = PointSize/gl_Position.w;  // 距離に応じて点の大きさを変更する
}

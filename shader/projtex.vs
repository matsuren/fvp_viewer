#version 430

layout (location = 0) in vec3 VertexPosition;
layout (location = 1) in vec3 VertexNormal;

out vec3 EyeNormal;       // Normal in eye coordinates
out vec4 EyePosition;     // Position in eye coordinates

uniform vec3 WorldCameraPosition;
uniform mat4 ModelViewMatrix;
uniform mat4 ModelMatrix;
uniform mat3 NormalMatrix;
uniform mat4 MVP;

// for OCamCalib
out vec4 point3Ds_from_fisheye[4];
uniform mat4 FisheyeCameraViews[4];
uniform int CAMERA_NUM;


///////////////////////////////////////////////
// main
///////////////////////////////////////////////
void main()
{
    vec4 pos4 = vec4(VertexPosition,1.0);

    EyeNormal = normalize(NormalMatrix * VertexNormal);
    EyePosition = ModelViewMatrix * pos4;
    gl_Position = MVP * pos4;
    
    // for OCamCalib
    for(int i =0; i<CAMERA_NUM ; i++){
      point3Ds_from_fisheye[i]=  FisheyeCameraViews[i] * pos4;
    }

}

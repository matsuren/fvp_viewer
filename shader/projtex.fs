#version 430
#extension GL_EXT_texture_array: enable

#define M_PI 3.1415926535897932384626433832795

in vec3 EyeNormal;       // Normal in eye coordinates
in vec4 EyePosition;     // Position in eye coordinates

layout( binding=0 ) uniform sampler2DArray FisheyeTexs;

layout( location = 0 ) out vec4 FragColor;

// for OCamCalib
in vec4 point3Ds_from_fisheye[4];

struct OCamCalibParam {
  float xc;
  float yc;
  float c;
  float d;
  float e;
  float width;
  float height;
  float invpol[14];
  float pol[6];
};
uniform OCamCalibParam OCamParams[4];

uniform int CAMERA_NUM;

///////////////////////////////////////////////
// for OCamCalib
// coord is different from offical ocamcalib
// return point2D[0]:m_col point2D[1]:m_row  
///////////////////////////////////////////////
float world2cam(vec4 point3D, OCamCalibParam param, out vec2 point2D)
{
  point3D.xy = point3D.yx;
  point3D.z = -point3D.z;
  float point3D_norm = sqrt(point3D.x * point3D.x + point3D.y * point3D.y);
  float theta = atan(point3D.z/point3D_norm);

  if(theta > radians(0.0) ){
    point2D = vec2(0.0); 
    return theta;
  }

  if(point3D_norm != 0){
    float invnorm = 1.0/point3D_norm;
    float rho = param.invpol[0];
    float t_i = 1.0;
    float t = theta;
    
    for(int i = 1; i < 14; i++){
      t_i *= t;
      rho += t_i*param.invpol[i];
    }
    
    float x = point3D.x * invnorm*rho;
    float y = point3D.y * invnorm*rho;
    // 注意 OCamCalibの座標系
    point2D.x = x * param.c + y * param.d + param.xc;
    point2D.y = x * param.e + y + param.yc;

  }else{
    // 注意 OCamCalibの座標系
    point2D.x = param.xc;
    point2D.y = param.yc;
  }
  
  // opengl u,v coordinates' origin : lower left
  point2D.x =  point2D.x / (param.height);
  point2D.y = point2D.y / (param.width);

  // ocamcalib coordinates x : height, y : width
  point2D.xy = point2D.yx;

  return theta;
}

///////////////////////////////////////////////
// blend function
///////////////////////////////////////////////
vec4 blend_no_alpha(float theta_array[4], vec4 texture_colors[4])
{
  vec4 return_color;
  // blend color
  float min_theta = 1.0;
  for(int i=0; i<4; i++){
    if(theta_array[i] > 0)
      continue;
    if(min_theta > theta_array[i]){
        min_theta = theta_array[i];
        return_color = texture_colors[i];
      }
  }
  return return_color;

}
// alpha
vec4 blend_alpha_mthesis(float theta_array[4], vec4 texture_colors[4])
{
  int test_CAMERA_NUMBER = 4;
  vec4 return_color = vec4(0.0);
  // blend color
  float first_min_theta = 1.0;
  int first_min_id = 0;
  float second_min_theta = 1.0;
  int second_min_id = 0;

  float sum_weight = 0.0;
  float weight[4] = {0.0, 0.0, 0.0, 0.0};

  for(int i=0; i<test_CAMERA_NUMBER; i++){
    if(theta_array[i] > 0)
      continue;
    if(first_min_theta > theta_array[i]){
      second_min_theta = first_min_theta;
      first_min_theta = theta_array[i];
      second_min_id = first_min_id;
      first_min_id = i; 
    }else if(second_min_theta > theta_array[i]){
      second_min_theta = theta_array[i];
      second_min_id = i; 
    }
  }

  float margin_radius = radians(15);
  float diff_radius = second_min_theta - first_min_theta;
  // blend two image
  if(diff_radius < margin_radius ){
    weight[first_min_id] = 0.5 * sin(M_PI /2.0 * diff_radius / margin_radius) + 0.5;
    weight[second_min_id] = 1.0 - weight[first_min_id];
  }
  // just show one image
  else{
    weight[first_min_id] = 1.0;
  }

  // weighted add
  for(int i=0; i<test_CAMERA_NUMBER; i++){
    sum_weight += weight[i];
  }
  for(int i=0; i<test_CAMERA_NUMBER;i++){
   return_color += texture_colors[i] * weight[i]/sum_weight;
  }
  return_color.w = 1.0;
  return return_color;

}

///////////////////////////////////////////////
// main
///////////////////////////////////////////////
void main() {
  
  // for OCamCalib
  vec4 point3D;
  vec2 point2D;
  float theta_array[4];
  vec4 texture_colors[4];
  float distance_from_cam[4];

  // calculate texture color of each camera
  for(int i=0; i<4; i++){
    point3D = vec4(point3Ds_from_fisheye[i].xyz, 1.0);
    theta_array[i] = world2cam(point3D, OCamParams[i], point2D);
    texture_colors[i] = texture2DArray( FisheyeTexs, vec3(point2D, float(i))); 
    distance_from_cam[i] = length(point3Ds_from_fisheye[i].xyz);
  }

   FragColor = blend_alpha_mthesis(theta_array, texture_colors);
   //FragColor = blend_no_alpha(theta_array, texture_colors);

   if(theta_array[0]>0 && theta_array[1]>0 && theta_array[2]>0 &&
       theta_array[3]>0){
       FragColor = vec4(0.0, 0.0, 0.0, 1.0);
   }

}

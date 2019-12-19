#version 430

in vec3 Position;
in vec3 Normal;
in vec2 TexCoord;
in vec4 Color;

struct LightInfo {
  vec4 Position; // Light position in eye coords.
  vec4 La;       // Ambient light intensity
  vec4 Ld;       // Diffuse light intensity
  vec4 Ls;       // Specular light intensity
};
uniform LightInfo Light;

struct MaterialInfo {
  vec4 Ka;            // Ambient reflectivity
  vec4 Kd;            // Diffuse reflectivity
  vec4 Ks;            // Specular reflectivity
  float Shininess;    // Specular shininess factor
};
uniform MaterialInfo Material;

layout (binding=0) uniform sampler2D Tex;
uniform bool TexFlag;
uniform bool RenderPointsFlag;

layout (location=0) out vec4 FragColor;

// ---------------------------------------------------------
void phongModelAmbiDiffSpec(out vec4 ambAndDiff, out vec4 spec ){
  vec3 n = Normal;
  if( !gl_FrontFacing ) n = -n;
  vec3 s = normalize(vec3(Light.Position) - Position);
  vec3 v = normalize(-Position.xyz);
  vec3 r = reflect( -s, n );
  float sDotN = max( dot(s,n), 0.0 );
  vec4 ambient = Light.La * Material.Ka * 0.2;
  vec4 diffuse = Light.Ld * Material.Kd * sDotN;
  if( sDotN > 0.0 )
    spec = Light.Ls * Material.Ks *
      pow( max( dot(r,v), 0.0 ), Material.Shininess );
  else
    spec = vec4(0,0,0,1);

  ambAndDiff = diffuse + ambient;
}
// ---------------------------------------------------------
void main() {
  vec4 ambAndDiff, spec;
  phongModelAmbiDiffSpec(ambAndDiff, spec);

  float mix_texture = 0.8;
  if(TexFlag){
    /* if(false){ */
    vec4 texColor = texture( Tex, TexCoord );
    FragColor = mix_texture * texColor + (1-mix_texture) * texColor * ambAndDiff + spec;
    /* FragColor = texColor; */

  }else{
    FragColor = ambAndDiff + spec;
  }
  /* FragColor = vec4(Color, 1.0); */
  /* vec4 texColor = texture( Tex, TexCoord ); */
  /* FragColor = texColor;                     */
  if(RenderPointsFlag){
    FragColor = Color;
  }
    /* FragColor = Material.Kd; */
    /* FragColor = Light.Ls;    */
}

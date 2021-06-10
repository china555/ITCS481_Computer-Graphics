/*____________________________________________________________________
|
| File: main.cpp
|
| Description: Hermite curve drawing.
|
| Functions: 
|
| (C) Copyright 2007 Mores Prachyabrued
|___________________________________________________________________*/

// Enable older CRT functions (such as strcpy) without warnings from vc8 (vc 2005 .net)
#if _MSC_VER >= 1400
#define _CRT_SECURE_NO_DEPRECATE // shut up the vc8 compiler
#define _CRT_NONSTDC_NO_DEPRECATE
#endif

/*___________________
|
| Include Files
|__________________*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

/*___________________
|
| Constants
|__________________*/

const int WIN_WIDTH_INIT = 800;
const int WIN_HEIGHT_INIT = 600;
int cam_id = 0;
const float C_TSTEP = 0.0005f;       // Curve time step (for rendering curve)
const gmtl::Vec3f WORLD_UP(0, 1, 0); // World up axis (Y)
bool is_d_on = true;
const int PT_NB = 6; // Number of input points (equals #tangents and #segments)
const float P_LENGTH = 3;
const gmtl::Point3f input_pts[PT_NB] =
    {
        gmtl::Point3f(-360, -50, 300),
        gmtl::Point3f(0, -75, 0),
        gmtl::Point3f(-360, -25, -300),
        gmtl::Point3f(0, 0, -300),
        gmtl::Point3f(-360, -35, 0),
        gmtl::Point3f(0, -75, 300),
};
const gmtl::Point3f input_pts2[PT_NB] =
    {
        gmtl::Point3f(560, 25, 300),
        gmtl::Point3f(200, 0, 0),
        gmtl::Point3f(560, 50, -300),
        gmtl::Point3f(200, 75, -300),
        gmtl::Point3f(560, 40, 0),
        gmtl::Point3f(200, 0, 300),

};
bool fullScreen = false;
gmtl::Matrix44f MMAT;      // Basis matrix for Hermite curve form (const)
gmtl::Matrix44f pposeadj2; // Adjusted plane coordinate system that the (plane) camera will be attached to

enum KeyModifier
{
  KM_SHIFT = 0,
  KM_CTRL,
  KM_ALT
};
bool On = true;
bool On1 = true;
/*___________________
|
| Type Definitions
|__________________*/

// Particle structure storing position, velocity, and other properties
typedef struct _MyParticle
{
  gmtl::Point3f p; // Position
  gmtl::Vec3f v;   // Velocity
  float m;         // Mass
  int ttl;         // Time to live, decremented each iteration
  int ttl_init;    // Initial ttl, used to fade color as the age increases
} MyParticle;
// Textures
enum TextureID
{
  TID_SKYBACK = 0,
  TID_SKYFRONT = 1,
  TID_SKYLEFT,
  TID_SKYRIGHT,
  TID_SKYBOTTOM,
  TID_SKYTOP,
  TEXTURE_NB
}; // Texture IDs, with the last ID indicating the total number of textures
/*___________________
|
| Global variables
|__________________*/

// camera w.r.t. plane
float distance[3] = {600.0f, 600.0f, 600.0f};  // Distance of the camera from world's origin
float elevation[3] = {-15.0f, -15.0f, -15.0f}; // Elevation of the camera (in degs)
float azimuth[3] = {180.0f, 180.0f, 180.0f};   // Azimuth of the camera (in degs)

// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3] = {false, false, false};
bool kmodifiers[3] = {false, false, false};

const gmtl::Point3f PROPELLER_POS(0, -0.3, (-P_LENGTH / 2) * 3.1); // Propeller position on the plane (w.r.t. plane's frame)
float PROPELLER_ROTATION = 10.0f;
const float CAM_FOV = 90.0f; // Field of view in degs

gmtl::Vec3f tangents[PT_NB], tangents2[PT_NB]; // Tangent at each input point
float s_tan = 1.0f;                            // A scaling factor for tangents

gmtl::Matrix44f Cmats[PT_NB], Cmats2[PT_NB]; // Coefficient matrix (C) for each Hermite curve segment (It's actually 4x3, ignore last column)

gmtl::Matrix44f ppose;                                      // The plane's pose
gmtl::Matrix44f pposeadj;                                   // Adjusted plane coordinate system that the (plane) camera will be attached to
gmtl::Matrix44f pposeadj_inv;                               // Adjusted plane coordinate system (plane's camera is attached to this frame), inverted
int ps = 0;                                                 // The segment in which the plane currently belongs
float pt = 0;                                               // The current t-value for the plane
float pdt = 0.02f;                                          // delta_t for the plane
const int PARTICLE_NB = 40;                                 // Number of particles
MyParticle particles[PARTICLE_NB], particles2[PARTICLE_NB]; // Array of particles
// Skybox
const float SB_SIZE = 1000.0f; // Skybox dimension
gmtl::Point4f light_pos(0.0, 20.0, 20.0, 1.0);
// Lighting
const GLfloat NO_LIGHT[] = {0.0, 0.0, 0.0, 1.0};
const GLfloat AMBIENT_LIGHT[] = {0.1, 0.1, 0.1, 1.0};
const GLfloat DIFFUSE_LIGHT[] = {0.5, 0.5, 0.5, 1.0};
const GLfloat SPECULAR_LIGHT[] = {0.5, 0.5, 0.5, 1.0};
//////////////////////////////////////
gmtl::Matrix44f ppose2;
const float R_MAX = 2.5f * 1200.0f; // Expected maximum magnitude of local rightward component of plane's acceleration

float VMAG_MEAN = 100.0f; // Velocity
float VMAG_STD = 25.0f;
float VDIR_STD = 0.25f;

const int TTL_BASE = 200; // Time to live
const int TTL_OFFSET = 50;

float SMOKE_SIZE = 15; // Smoke size

const float S_TSTEP = 0.001f; // Simulation time step (for particle update)

gmtl::Vec3f GRAVITY(0, -100.0f, 0); // w.r.t. world, can be upward for smoke

gmtl::Vec3f V_WIND(100, 0, 0); // Wind velocity and drag coefficient (K)
float K_COEF = 1.0f;
GLuint texture, texture2;
// Rendering option
bool render_curve = true;
bool render_constraint = false;

gmtl::Point4f plane_p, plane1Position, ppPosition;   // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q, plane1Quaternion, ppQuaternion; // Quaternion

// Quaternions to rotate plane
gmtl::Quatf zrotp_q, xrotp_q, yrotp_q, ppzrotp_q; // Positive and negative Z rotations
gmtl::Quatf zrotn_q, xrotn_q, yrotn_q, ppzrotn_q;
GLuint textures[TEXTURE_NB];
gmtl::Matrix44f pposeadj_inv2; // Adjusted plane coordinate system (plane's camera is attached to this frame), inverted

/*___________________
|
| Function Prototypes
|__________________*/

void Init();
void Init_AParticle(MyParticle &par, MyParticle &par2);
void Compute_Tangents();
void Compute_Coefficiences();
void Display_Func();
void Idle_Func();
void DrawSkybox(const float s);
void Update_Plane();
void Update_PSystem();
void Keyboard_Func(unsigned char key, int x, int y);
void Reshape_Func(int width, int height);
void Mouse_Func(int button, int state, int x, int y);
void Motion_Func(int x, int y);
void Draw_World_Axes();
void Draw_Path();
void Draw_Rocket();
void Draw_Particles();
float FastGauss(float mean, float std);
void SetLight(const gmtl::Point4f &pos, const bool is_ambient, const bool is_diffuse, const bool is_specular);
void LoadPPM(char *fname, unsigned int *w, unsigned int *h, unsigned char **data, int mallocflag);

/*____________________________________________________________________
|
| Function: main
|
| Input:
| Output: Program entry point.
|___________________________________________________________________*/

int main(int argc, char **argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(WIN_WIDTH_INIT, WIN_HEIGHT_INIT);
  glutCreateWindow("Hermite Curve Drawing");

  glutDisplayFunc(Display_Func);
  glutIdleFunc(Idle_Func);
  glutKeyboardFunc(Keyboard_Func);
  glutReshapeFunc(Reshape_Func);
  glutMouseFunc(Mouse_Func);
  glutMotionFunc(Motion_Func);

  Init();

  glutMainLoop();

  return 0;
}

/*____________________________________________________________________
|
| Function: Init
|
| Input:
| Output: Initialization routine.
|___________________________________________________________________*/

void Init()
{
  int i;
  unsigned int texwidth, texheight;
  unsigned char *imagedata;
  unsigned char *img_data; // Texture image data
  unsigned int width;      // Texture width
  unsigned int height;     // Texture height

  // OpenGL
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);

  // Inits Hermite basis matrix
  MMAT.set(
      2, -2, 1, 1,
      -3, 3, -2, -1,
      0, 0, 1, 0,
      1, 0, 0, 0);

  // Inits tangents and coefficient matrices
  Compute_Tangents();
  Compute_Coefficiences();

  // Init adjusted plane-coordinate-system that the camera is attached to (HACK! by calling Update_Plane here)
  Update_Plane();

  // Inits particle system
  for (i = 0; i < PARTICLE_NB; i++)
  {
    Init_AParticle(particles[i], particles2[i]);
  }
  //|___________________________________________________________________
  //|
  //| Setup lighting
  //|___________________________________________________________________

  // Disable global ambient
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, NO_LIGHT);

  // NOTE: for specular reflections, the "local viewer" model produces better
  // results than the default, but is slower. The default would not use the correct
  // vertex-to-eyepoint vector, treating it as always parallel to Z.
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

  // Enable two sided lighting
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  // Enable lighting
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  /*____________________________________________________________________
|
| Load texture
|___________________________________________________________________*/

  // describe how data will be stored in memory
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  // generate a new "texture object" and select it for setup
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);

  // load an image into memory
  LoadPPM("10.ppm", &texwidth, &texheight, &imagedata, 1);
  // describe the image to the graphics system as a texture map
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0,
               GL_RGB, GL_UNSIGNED_BYTE, imagedata);
  free(imagedata);

  // select methods for "scaling" a texture region to a pixel
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // select the method for combining texture color with the lighting equation
  // (look up the third parameter)
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  ///////////////////////////////////////////////////////////Particle 2//////////////////////////////////////////////////////////////////////
  glGenTextures(1, &texture2);
  glBindTexture(GL_TEXTURE_2D, texture2);

  // load an image into memory
  LoadPPM("55.ppm", &texwidth, &texheight, &imagedata, 1);
  // describe the image to the graphics system as a texture map
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0,
               GL_RGB, GL_UNSIGNED_BYTE, imagedata);
  free(imagedata);

  // select methods for "scaling" a texture region to a pixel
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // select the method for combining texture color with the lighting equation
  // (look up the third parameter)
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

  // NOTE: to have another texture map, generate another texture object and
  //       repeat the setup steps. To select which texture is being applied
  //       during drawing, use glBindTexture() to select.
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // Select the method for combining texture color with the lighting equation
  // (look up the third parameter)
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

  // Generate and setup texture objects
  glGenTextures(TEXTURE_NB, textures);

  // Skybox back wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBACK]);
  LoadPPM("1.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Skybox FRONT wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYFRONT]);
  LoadPPM("4.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Skybox left wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYLEFT]);
  LoadPPM("4.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  // Skybox right wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYRIGHT]);
  LoadPPM("1.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Skybox bottom wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBOTTOM]);
  LoadPPM("2.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  // Skybox top wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYTOP]);
  LoadPPM("6.ppm", &width, &height, &img_data, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  free(img_data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

/*____________________________________________________________________
|
| Function: Init_AParticle
|
| Input:
| Output: Init a single particle.
|___________________________________________________________________*/

void Init_AParticle(MyParticle &par, MyParticle &par2)
{
  gmtl::Vec3f v_dir;

  // position (= plane position)
  par.p.set(ppose[0][3], ppose[1][3], ppose[2][3]);
  par2.p.set(ppose2[0][3], ppose2[1][3], ppose2[2][3]);

  // velocity (consider plane's -Z axis as mean velocity direction)
  v_dir.set(-ppose[0][2], -ppose[1][2], -ppose[2][2]);
  v_dir.set(-ppose2[0][2], -ppose2[1][2], -ppose2[2][2]);

  par.v.set(
      FastGauss(v_dir[0], VDIR_STD),
      FastGauss(v_dir[1], VDIR_STD),
      FastGauss(v_dir[2], VDIR_STD));
  par.v = FastGauss(VMAG_MEAN, VMAG_STD) * par.v;

  par2.v.set(
      FastGauss(v_dir[0], VDIR_STD),
      FastGauss(v_dir[1], VDIR_STD),
      FastGauss(v_dir[2], VDIR_STD));
  par2.v = FastGauss(VMAG_MEAN, VMAG_STD) * par2.v;

  // mass
  par.m = 1;
  par2.m = 1;

  // ttl
  par.ttl = par.ttl_init = (rand() % TTL_BASE) + TTL_OFFSET;
  par2.ttl = par2.ttl_init = (rand() % TTL_BASE) + TTL_OFFSET;
}

/*____________________________________________________________________
|
| Function: Compute_Tangents
|
| Input:
| Output: Computes tangents from input points and the scale factor
|___________________________________________________________________*/

void Compute_Tangents()
{
  int i;
  int prev, next; // Adjacent points

  for (i = 0; i < PT_NB; i++)
  {
    // Compute adjacent points
    if (i == 0)
    { // First point
      prev = PT_NB - 1;
      next = i + 1;
    }
    else if (i == PT_NB - 1)
    { // Last point
      prev = i - 1;
      next = 0;
    }
    else
    { // Interior point
      prev = i - 1;
      next = i + 1;
    }

    tangents[i] = s_tan * (input_pts[next] - input_pts[prev]);
    tangents2[i] = s_tan * (input_pts2[next] - input_pts2[prev]);
  }
}

/*____________________________________________________________________
|
| Function: Compute_Coefficiences
|
| Input:
| Output: Computes coefficiences matrices for curve segments
|___________________________________________________________________*/

void Compute_Coefficiences()
{
  int i;
  int n;                // Next input point (index)
  gmtl::Matrix44f Gmat; // Geometry matrix (It's actually 4x3, ignore last column)
  gmtl::Matrix44f Gmat2;
  for (i = 0; i < PT_NB; i++)
  {
    if (i == PT_NB - 1)
    {
      n = 0;
    }
    else
    {
      n = i + 1;
    }

    Gmat.set(
        input_pts[i][0], input_pts[i][1], input_pts[i][2], 0,
        input_pts[n][0], input_pts[n][1], input_pts[n][2], 0,
        tangents[i][0], tangents[i][1], tangents[i][2], 0,
        tangents[n][0], tangents[n][1], tangents[n][2], 0);
    Gmat2.set(
        input_pts2[i][0], input_pts2[i][1], input_pts2[i][2], 0,
        input_pts2[n][0], input_pts2[n][1], input_pts2[n][2], 0,
        tangents2[i][0], tangents2[i][1], tangents2[i][2], 0,
        tangents2[n][0], tangents2[n][1], tangents2[n][2], 0);

    Cmats[i] = MMAT * Gmat;
    Cmats2[i] = MMAT * Gmat2;
  }
}

/*____________________________________________________________________
|
| Function: Display_Func
|
| Input:
| Output: GLUT display callback function.
|___________________________________________________________________*/

void Display_Func(void)
{
  // Clear screen & depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  switch (cam_id)
  {
  case 0:
    // For the world-relative camera
    glTranslatef(0, 0, -distance[0]);
    glRotatef(-elevation[0], 1, 0, 0);
    glRotatef(-azimuth[0], 0, 1, 0);
    break;

  case 1:
    // For plane2's camera
    glTranslatef(0, 0, -distance[1]);
    glRotatef(-elevation[1], 1, 0, 0);
    glRotatef(-azimuth[1], 0, 1, 0);
    glMultMatrixf(pposeadj_inv.mData);
    break;
  case 2:
    // For plane1's camera
    glTranslatef(0, 0, -distance[2]);
    glRotatef(-elevation[2], 1, 0, 0);
    glRotatef(-azimuth[2], 0, 1, 0);
    glMultMatrixf(pposeadj_inv2.mData);
    break;

    // TODO: Add case for the plane1's camera
  }
  SetLight(light_pos, true, is_d_on, true);
  // Add view matrix

  DrawSkybox(SB_SIZE);
  if (On1)
  {
    Draw_World_Axes();
  }
  Draw_Path();
  Draw_Rocket();
  Draw_Rocket();
  Draw_Particles();

  // Display!
  glutSwapBuffers();
}

/*____________________________________________________________________
|
| Function: Idle_Func
|
| Input:
| Output: GLUT idle callback function. Called for each simulation step.
|___________________________________________________________________*/

void Idle_Func()
{
  Update_Plane();
  Update_PSystem();
  glutPostRedisplay();
}

/*____________________________________________________________________
|
| Function: Update_Plane
|
| Input:
| Output: Called every simulation step to update the plane's pose.
|___________________________________________________________________*/

void Update_Plane()
{
  float x, y, z;            // Plane's coordinates
  gmtl::Vec3f xa, ya, za;   // Plane's axes
  gmtl::Vec3f ac;           // Acceleration vector
  float r;                  // Local rightward component of acceleration
  float ra;                 // Roll angle
  gmtl::AxisAnglef roll_aa; // Local roll in axis angle and matrix forms
  gmtl::Matrix44f roll_mat;
  gmtl::Vec3f zadj;

  float x2, y2, z2;          // Plane's coordinates
  gmtl::Vec3f xa2, ya2, za2; // Plane's axes
  gmtl::Vec3f ac2;           // Acceleration vector
  float r2;                  // Local rightward component of acceleration
  float ra2;                 // Roll angle
  gmtl::AxisAnglef roll_aa2; // Local roll in axis angle and matrix forms
  gmtl::Matrix44f roll_mat2;
  gmtl::Vec3f zadj2;

  /*____________________________________________________________________
|
| Update t and possibly current segment
|___________________________________________________________________*/

  pt += pdt;
  if (pt > 1)
  {
    pt -= 1;
    ps++;
    if (ps == PT_NB)
    {
      ps = 0;
    }
  }

  /*____________________________________________________________________
|
| Compute plane's new position by evaluating the polynomials (use Horner's rule for slight speedup)
|___________________________________________________________________*/

  x = ((Cmats[ps][0][0] * pt + Cmats[ps][1][0]) * pt + Cmats[ps][2][0]) * pt + Cmats[ps][3][0];
  y = ((Cmats[ps][0][1] * pt + Cmats[ps][1][1]) * pt + Cmats[ps][2][1]) * pt + Cmats[ps][3][1];
  z = ((Cmats[ps][0][2] * pt + Cmats[ps][1][2]) * pt + Cmats[ps][2][2]) * pt + Cmats[ps][3][2];

  x2 = ((Cmats2[ps][0][0] * pt + Cmats2[ps][1][0]) * pt + Cmats2[ps][2][0]) * pt + Cmats2[ps][3][0];
  y2 = ((Cmats2[ps][0][1] * pt + Cmats2[ps][1][1]) * pt + Cmats2[ps][2][1]) * pt + Cmats2[ps][3][1];
  z2 = ((Cmats2[ps][0][2] * pt + Cmats2[ps][1][2]) * pt + Cmats2[ps][2][2]) * pt + Cmats2[ps][3][2];

  /*____________________________________________________________________
|
| Compute plane's orientation
|___________________________________________________________________*/

  // Compute direction of motion (z) by evaluating the polynomial derivatives (use Horner's rule for slight speedup)
  za[0] = (3 * Cmats[ps][0][0] * pt + 2 * Cmats[ps][1][0]) * pt + Cmats[ps][2][0];
  za[1] = (3 * Cmats[ps][0][1] * pt + 2 * Cmats[ps][1][1]) * pt + Cmats[ps][2][1];
  za[2] = (3 * Cmats[ps][0][2] * pt + 2 * Cmats[ps][1][2]) * pt + Cmats[ps][2][2];
  za2[0] = (3 * Cmats2[ps][0][0] * pt + 2 * Cmats2[ps][1][0]) * pt + Cmats2[ps][2][0];
  za2[1] = (3 * Cmats2[ps][0][1] * pt + 2 * Cmats2[ps][1][1]) * pt + Cmats2[ps][2][1];
  za2[2] = (3 * Cmats2[ps][0][2] * pt + 2 * Cmats2[ps][1][2]) * pt + Cmats2[ps][2][2];
  gmtl::normalize(za);
  gmtl::normalize(za2);

  // Compute lateral axis (x)
  gmtl::cross(xa, WORLD_UP, za);
  gmtl::normalize(xa);

  gmtl::cross(xa2, WORLD_UP, za2);
  gmtl::normalize(xa2);

  // Compute up axis (x)
  gmtl::cross(ya, za, xa);
  gmtl::normalize(ya);

  gmtl::cross(ya2, za2, xa2);
  gmtl::normalize(ya2);

  /*____________________________________________________________________
|
| Compute banked turn (local roll)
|___________________________________________________________________*/

  // Compute acceleration vector
  ac[0] = 6 * Cmats[ps][0][0] * pt + 2 * Cmats[ps][1][0];
  ac[1] = 6 * Cmats[ps][0][1] * pt + 2 * Cmats[ps][1][1];
  ac[2] = 6 * Cmats[ps][0][2] * pt + 2 * Cmats[ps][1][2];

  ac2[0] = 6 * Cmats2[ps][0][0] * pt + 2 * Cmats2[ps][1][0];
  ac2[1] = 6 * Cmats2[ps][0][1] * pt + 2 * Cmats2[ps][1][1];
  ac2[2] = 6 * Cmats2[ps][0][2] * pt + 2 * Cmats2[ps][1][2];

  // Compute local rightward component of acceleration
  r = gmtl::dot(ac, xa);

  r2 = gmtl::dot(ac2, xa2);

  // Clamp r to R_MAX
  if (r > R_MAX)
  {
    r = R_MAX;
  }
  else if (r < -R_MAX)
  {
    r = -R_MAX;
  }

  if (r2 > R_MAX)
  {
    r2 = R_MAX;
  }
  else if (r2 < -R_MAX)
  {
    r2 = -R_MAX;
  }

  // Compute roll angle
  ra = asin(r / R_MAX);

  ra2 = asin(r2 / R_MAX);

  //printf("%.2f\n", r);

  /*____________________________________________________________________
|
| Update plane's pose
|___________________________________________________________________*/

  ppose.set(
      xa[0], ya[0], za[0], x,
      xa[1], ya[1], za[1], y,
      xa[2], ya[2], za[2], z,
      0, 0, 0, 1);
  ppose.setState(gmtl::Matrix44f::AFFINE);

  ppose2.set(
      xa2[0], ya2[0], za2[0], x2,
      xa2[1], ya2[1], za2[1], y2,
      xa2[2], ya2[2], za2[2], z2,
      0, 0, 0, 1);
  ppose2.setState(gmtl::Matrix44f::AFFINE);

  // Compute local roll (rotation about longitudinal axis (z))
  roll_aa.set(ra, za[0], za[1], za[2]);
  gmtl::set(roll_mat, roll_aa);

  roll_aa2.set(ra2, za2[0], za2[1], za2[2]);
  gmtl::set(roll_mat2, roll_aa2);

  // Apply local roll
  ppose = ppose * roll_mat;

  ppose2 = ppose2 * roll_mat2;

  /*____________________________________________________________________
|
| Compute adjusted plane coordinate system and its inverse
|___________________________________________________________________*/

  gmtl::cross(zadj, xa, WORLD_UP);
  gmtl::normalize(zadj);

  gmtl::cross(zadj2, xa2, WORLD_UP);
  gmtl::normalize(zadj2);

  pposeadj.set(
      xa[0], WORLD_UP[0], zadj[0], x,
      xa[1], WORLD_UP[1], zadj[1], y,
      xa[2], WORLD_UP[2], zadj[2], z,
      0, 0, 0, 1);
  pposeadj.setState(gmtl::Matrix44f::AFFINE);

  gmtl::invert(pposeadj_inv, pposeadj);

  pposeadj2.set(
      xa2[0], WORLD_UP[0], zadj2[0], x2,
      xa2[1], WORLD_UP[1], zadj2[1], y2,
      xa2[2], WORLD_UP[2], zadj2[2], z2,
      0, 0, 0, 1);
  pposeadj2.setState(gmtl::Matrix44f::AFFINE);

  gmtl::invert(pposeadj_inv2, pposeadj2);
}

/*____________________________________________________________________
|
| Function: Update_PSystem
|
| Input:
| Output: Called every simulation step to update the particle system.
|___________________________________________________________________*/

void Update_PSystem()
{
  int i;
  gmtl::Vec3f F;  // Net force
  gmtl::Vec3f a;  // Acceleration
  gmtl::Vec3f F2; // Net force
  gmtl::Vec3f a2; // Acceleration

  for (i = 0; i < PARTICLE_NB; i++)
  {
    // Life is shorten by one time unit
    particles[i].ttl--;

    particles2[i].ttl--;

    if (particles[i].ttl > 0 && particles2[i].ttl > 0)
    { // Still active
      // Update position
      particles[i].p += S_TSTEP * particles[i].v;
      particles2[i].p += S_TSTEP * particles2[i].v;

      // Compute net force from gravity and vicous drag (from wind)
      F = (particles[i].m * GRAVITY) - (K_COEF * (particles[i].v - V_WIND));
      F2 = (particles2[i].m * GRAVITY) - (K_COEF * (particles2[i].v - V_WIND));

      // Calculate acceleration from froce
      a = F / particles[i].m;
      a2 = F2 / particles2[i].m;

      // Update velocity
      particles[i].v += S_TSTEP * a;
      particles2[i].v += S_TSTEP * a2;
    }
    else
    { // Died, make it reborn
      Init_AParticle(particles[i], particles2[i]);
    }
  }
}

/*____________________________________________________________________
|
| Function: Keyboard_Func
|
| Input:
| Output: GLUT keyboard callback function.
|___________________________________________________________________*/

void Keyboard_Func(unsigned char key, int x, int y)
{
  switch ((char)key)
  {
  case 'q': // Toggle curve drawing
    render_curve = !render_curve;
    break;
  case 'w': // Toggle constraint drawing
    render_constraint = !render_constraint;
    break;
  case 'e': // Select camera to view
    cam_id = (cam_id + 1) % 3;
    printf("View camera = %d\n", cam_id);
    break;

    //| Propeller controls (subpart)
    //|____________________________________________________________________

  case 't':
    VMAG_MEAN += 100;
    printf("VMAG_MEAN = %f \n", VMAG_MEAN);
    break;
  case 'y':
    VMAG_MEAN -= 100;
    printf("VMAG_MEAN = %f \n", VMAG_MEAN);
    break;
  case 'u':
    SMOKE_SIZE += 1;
    printf("SMOKE_SIZE = %f \n", SMOKE_SIZE);
    break;
  case 'i':
    if (SMOKE_SIZE >= 1)
    {
      SMOKE_SIZE -= 1;
      printf("SMOKE_SIZE = %f \n", SMOKE_SIZE);
    }
    break;
  case 'o':
    K_COEF += 0.1;
    printf("K_COEF = %f \n", K_COEF);
    break;
  case 'p':
    K_COEF -= 0.1;
    printf("K_COEF = %f \n", K_COEF);
    break;
  case '[':
    GRAVITY[1] += 10;
    printf("GRAVITY[1] = %f \n", GRAVITY[1]);
    break;
  case ']':
    GRAVITY[1] -= 10;
    printf("GRAVITY[1] = %f \n", GRAVITY[1]);
    break;
  case '1':
    V_WIND[0] += 100;
    printf("V_WIND[0] = %f \n", V_WIND[0]);
    break;
  case '2':
    V_WIND[0] -= 100;
    printf("V_WIND[0] = %f \n", V_WIND[0]);
    break;
  case '3':
    V_WIND[1] += 100;
    printf("V_WIND[1] = %f \n", V_WIND[1]);
    break;
  case '4':
    V_WIND[1] -= 100;
    printf("V_WIND[1] = %f \n", V_WIND[1]);
    break;
  case '5':
    V_WIND[2] += 100;
    printf("V_WIND[2] = %f \n", V_WIND[2]);
    break;
  case '6':
    V_WIND[2] -= 100;
    printf("V_WIND[2] = %f \n", V_WIND[2]);
    break;
  case '7':
    pdt += 0.1;
    printf("DeltaT = %f \n", pdt);
    break;
  case '8':
    pdt -= 0.1;
    printf("DeltaT = %f \n", pdt);
    break;
  case '9':
    s_tan += 0.1;
    printf("Tanget_Scale = %f \n", s_tan);
    break;
  case '0':
    s_tan -= 0.1;
    printf("Tanget_Scale = %f \n", s_tan);
    break;
  case 'a':
    On = !On;
    break;
  case 'f': //toggle screenmode
    fullScreen = !fullScreen;
    if (fullScreen)
    {
      glutFullScreen();
    }
    else
    {
      glutReshapeWindow(1200, 900);
      glutPositionWindow(0, 0);
    }
    break;
  case 's':
    On1 = !On1;
    break;
  default:
    break;
  }

  glutPostRedisplay();
}

/*____________________________________________________________________
|
| Function: Reshape_Func
|
| Input:
| Output: GLUT reshape callback function.
|___________________________________________________________________*/

void Reshape_Func(int width, int height)
{
  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, ((float)width) / height, 1.0f, 10000.0f);
}

//|____________________________________________________________________
//|
//| Function: Mouse_Func
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void Mouse_Func(int button, int state, int x, int y)
{
  int km_state;

  // Updates button's sate and mouse coordinates
  if (state == GLUT_DOWN)
  {
    mbuttons[button] = true;
    mx_prev = x;
    my_prev = y;
  }
  else
  {
    mbuttons[button] = false;
  }

  // Updates keyboard modifiers
  km_state = glutGetModifiers();
  kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
  kmodifiers[KM_CTRL] = km_state & GLUT_ACTIVE_CTRL ? true : false;
  kmodifiers[KM_ALT] = km_state & GLUT_ACTIVE_ALT ? true : false;

  //glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: Motion_Func
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void Motion_Func(int x, int y)
{
  int dx, dy, d;

  if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON])
  {
    // Computes distances the mouse has moved
    dx = x - mx_prev;
    dy = y - my_prev;

    // Updates mouse coordinates
    mx_prev = x;
    my_prev = y;

    // Hold left button to rotate camera
    if (mbuttons[GLUT_LEFT_BUTTON])
    {
      if (!kmodifiers[KM_CTRL])
      {
        elevation[cam_id] += dy; // Elevation update
      }
      if (!kmodifiers[KM_SHIFT])
      {
        azimuth[cam_id] += dx; // Azimuth update
      }
    }

    // Hold right button to zoom
    if (mbuttons[GLUT_RIGHT_BUTTON])
    {
      if (abs(dx) >= abs(dy))
      {
        d = dx;
      }
      else
      {
        d = -dy;
      }
      distance[cam_id] += d;
    }

    glutPostRedisplay(); // Asks GLUT to redraw the screen
  }
}

/*____________________________________________________________________
|
| Function: Draw_World_Axes
|
| Input:
| Output: Draw world axes.
|___________________________________________________________________*/

void Draw_World_Axes()
{
  glBegin(GL_LINES);
  // x-axis is red
  glColor3f(1.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(100.0, 0.0, 0.0);
  // y-axis is green
  glColor3f(0.0, 1.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 100.0, 0.0);
  // z-axis is blue
  glColor3f(0.0, 0.0, 1.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 100.0);
  glEnd();
}

/*____________________________________________________________________
|
| Function: Draw_Path
|
| Input:
| Output: Draws a path.
|___________________________________________________________________*/
void DrawSkybox(const float s)
{
  float s2 = s;

  // Turn on texture mapping and disable lighting
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  // Back wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBACK]); // Specify which texture will be used
  glBegin(GL_QUADS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-s2, -s2, -s2);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(s2, -s2, -s2);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(s2, s2, -s2);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(-s2, s2, -s2);
  glEnd();
  // Front wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYFRONT]); // Specify which texture will be used
  glBegin(GL_QUADS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-s2, -s2, s2);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(s2, -s2, s2);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(s2, s2, s2);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(-s2, s2, s2);
  glEnd();

  // Left wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYLEFT]);
  glBegin(GL_QUADS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-s2, -s2, s2);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(-s2, -s2, -s2);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(-s2, s2, -s2);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(-s2, s2, s2);
  glEnd();
  // RIGHT wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYRIGHT]);
  glBegin(GL_QUADS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(s2, -s2, s2);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(s2, -s2, -s2);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(s2, s2, -s2);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(s2, s2, s2);
  glEnd();

  // Bottom wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBOTTOM]);
  glBegin(GL_QUADS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-s2, -s2, s2);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(s2, -s2, s2);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(s2, -s2, -s2);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(-s2, -s2, -s2);
  glEnd();
  // TOP wall
  glBindTexture(GL_TEXTURE_2D, textures[TID_SKYTOP]);
  glBegin(GL_QUADS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-s2, s2, s2);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(s2, s2, s2);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(s2, s2, -s2);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(-s2, s2, -s2);
  glEnd();

  // Turn off texture mapping and enable lighting
  glEnable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
}
void SetLight(const gmtl::Point4f &pos, const bool is_ambient, const bool is_diffuse, const bool is_specular)
{
  glLightfv(GL_LIGHT0, GL_POSITION, pos.mData);
  if (is_ambient)
  {
    glLightfv(GL_LIGHT0, GL_AMBIENT, AMBIENT_LIGHT);
  }
  else
  {
    glLightfv(GL_LIGHT0, GL_AMBIENT, NO_LIGHT);
  }
  if (is_diffuse)
  {
    glLightfv(GL_LIGHT0, GL_DIFFUSE, DIFFUSE_LIGHT);
  }
  else
  {
    glLightfv(GL_LIGHT0, GL_DIFFUSE, NO_LIGHT);
  }
  if (is_specular)
  {
    glLightfv(GL_LIGHT0, GL_SPECULAR, SPECULAR_LIGHT);
  }
  else
  {
    glLightfv(GL_LIGHT0, GL_SPECULAR, NO_LIGHT);
  }
}

void Draw_Path()
{
  int i;
  float x, y, z, t;
  glDisable(GL_LIGHTING);
  /*____________________________________________________________________
|
| Draw input points and tangents
|___________________________________________________________________*/

  if (render_constraint)
  {
    for (i = 0; i < PT_NB; i++)
    {
      glPushMatrix();
      glTranslatef(input_pts[i][0], input_pts[i][1], input_pts[i][2]);
      glColor3f(1, 0, 0);
      glutSolidSphere(5, 16, 16);
      glColor3f(1, 1, 0);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(tangents[i][0], tangents[i][1], tangents[i][2]);
      glEnd();
      glPopMatrix();

      glPushMatrix();
      glTranslatef(input_pts2[i][0], input_pts2[i][1], input_pts2[i][2]);
      glColor3f(1, 0, 0);
      glutSolidSphere(5, 16, 16);
      glColor3f(1, 1, 0);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(tangents2[i][0], tangents2[i][1], tangents2[i][2]);
      glEnd();
      glPopMatrix();
    }
  }

  /*____________________________________________________________________
|
| Draw Hermite curve segments using line strips
|___________________________________________________________________*/

  if (render_curve)
  {
    glColor3f(0, 1, 0);
    for (i = 0; i < PT_NB; i++)
    {
      // Draw each segment
      glBegin(GL_LINE_STRIP);
      for (t = 0; t <= 1; t += C_TSTEP)
      {
        // Simple polynomial evaluation
        //float t2 = t*t;
        //float t3 = t2*t;
        //x = Cmats[i][0][0]*t3 + Cmats[i][1][0]*t2 + Cmats[i][2][0]*t + Cmats[i][3][0];
        //y = Cmats[i][0][1]*t3 + Cmats[i][1][1]*t2 + Cmats[i][2][1]*t + Cmats[i][3][1];
        //z = Cmats[i][0][2]*t3 + Cmats[i][1][2]*t2 + Cmats[i][2][2]*t + Cmats[i][3][2];

        // Use Horner's rule for slight speedup
        x = ((Cmats[i][0][0] * t + Cmats[i][1][0]) * t + Cmats[i][2][0]) * t + Cmats[i][3][0];
        y = ((Cmats[i][0][1] * t + Cmats[i][1][1]) * t + Cmats[i][2][1]) * t + Cmats[i][3][1];
        z = ((Cmats[i][0][2] * t + Cmats[i][1][2]) * t + Cmats[i][2][2]) * t + Cmats[i][3][2];
        if (On)
        {
          glVertex3f(x, y, z);
        }
      }
      glEnd();

      glBegin(GL_LINE_STRIP);
      for (t = 0; t <= 1; t += C_TSTEP)
      {
        // Simple polynomial evaluation
        //float t2 = t*t;
        //float t3 = t2*t;
        //x = Cmats[i][0][0]*t3 + Cmats[i][1][0]*t2 + Cmats[i][2][0]*t + Cmats[i][3][0];
        //y = Cmats[i][0][1]*t3 + Cmats[i][1][1]*t2 + Cmats[i][2][1]*t + Cmats[i][3][1];
        //z = Cmats[i][0][2]*t3 + Cmats[i][1][2]*t2 + Cmats[i][2][2]*t + Cmats[i][3][2];

        // Use Horner's rule for slight speedup
        x = ((Cmats2[i][0][0] * t + Cmats2[i][1][0]) * t + Cmats2[i][2][0]) * t + Cmats2[i][3][0];
        y = ((Cmats2[i][0][1] * t + Cmats2[i][1][1]) * t + Cmats2[i][2][1]) * t + Cmats2[i][3][1];
        z = ((Cmats2[i][0][2] * t + Cmats2[i][1][2]) * t + Cmats2[i][2][2]) * t + Cmats2[i][3][2];
        if (On)
        {
          glVertex3f(x, y, z);
        }
      }
      glEnd();
    }
  }
  glEnable(GL_LIGHTING);
}

/*____________________________________________________________________
|
| Function: Draw_Rocket
|
| Input:
| Output: Draw a rocket (longitudial = z, lateral = x, up = y).
|___________________________________________________________________*/

void Draw_Rocket()
{
  glPushMatrix();

  glMultMatrixf(ppose.mData);
  glScalef(4, 4, 4); // If lighting enabled here, be sure to use GL_RESCALE_NORMAL

  // Draw body
  const float w = -3.0f;
  const float h = 1.5f;
  const float l = -3.0f;

  glBegin(GL_QUADS);
  ////////////////////////////////////////BODY///////////////////////////////////////////////
  glColor3f(0.9f, 0.9f, 0.9f);
  //right
  glVertex3f(w / 2, h / 2, -l);
  glVertex3f(w / 2, h / 2, l / 2);
  glVertex3f(w / 2, -h / 2, l / 2);
  glVertex3f(w / 2, -h / 2, -l);

  //Top
  glVertex3f(w / 4, h, -l);
  glVertex3f(-w / 4, h, -l);
  glVertex3f(-w / 4, h, l / 2);
  glVertex3f(w / 4, h, l / 2);

  //left
  glVertex3f(-w / 2, h / 2, -l);
  glVertex3f(-w / 2, h / 2, l / 2);
  glVertex3f(-w / 2, -h / 2, l / 2);
  glVertex3f(-w / 2, -h / 2, -l);

  //upper right
  glVertex3f(w / 4, h, -l);
  glVertex3f(w / 4, h, l / 2);
  glVertex3f(w / 2, h / 2, l / 2);
  glVertex3f(w / 2, h / 2, -l);

  //upper left
  glVertex3f(-w / 4, h, -l);
  glVertex3f(-w / 4, h, l / 2);
  glVertex3f(-w / 2, h / 2, l / 2);
  glVertex3f(-w / 2, h / 2, -l);

  glColor3f(0.8f, 0.0f, 0.2f);
  //lower left
  glVertex3f(-w / 2, -h / 2, -l);
  glVertex3f(-w / 2, -h / 2, l / 2);
  glVertex3f(-w / 4, -h, l / 2);
  glVertex3f(-w / 4, -h, -l);

  //lower right
  glVertex3f(w / 2, -h / 2, -l);
  glVertex3f(w / 2, -h / 2, l / 2);
  glVertex3f(w / 4, -h, l / 2);
  glVertex3f(w / 4, -h, -l);

  //lower
  glVertex3f(w / 4, -h, -l);
  glVertex3f(-w / 4, -h, -l);
  glVertex3f(-w / 4, -h, l / 2);
  glVertex3f(w / 4, -h, l / 2);
  /////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////Window////////////////////////////////////////////
  glColor3f(0.6f, 0.7f, 0.7f);

  //upper center slope window
  glVertex3f(w / 4, h / 4, -l * 1.4);
  glVertex3f(-w / 4, h / 4, -l * 1.4);
  glVertex3f(-w / 4, h, -l);
  glVertex3f(w / 4, h, -l);

  //upper right slope window
  glVertex3f(w / 4, h / 4, -l * 1.4);
  glVertex3f(w / 4, h, -l);
  glVertex3f(w / 2, h / 2, -l);
  glVertex3f(w / 2, h / 4, -l);

  //upper left slope window
  glVertex3f(-w / 4, h / 4, -l * 1.4);
  glVertex3f(-w / 4, h, -l);
  glVertex3f(-w / 2, h / 2, -l);
  glVertex3f(-w / 2, h / 4, -l);
  /////////////////////////////////////////////////////////

  ///////////////////////lowerwindow///////////////////////
  glColor3f(0.8f, 0.0f, 0.2f);

  //lower right slope window
  glVertex3f(w / 4, -h / 2, -l * 1.6);
  glVertex3f(w / 2, -h / 2, -l);
  glVertex3f(w / 4, -h, -l);
  glVertex3f(w / 4, -h, -l * 1.6);

  //lower left slope window
  glVertex3f(-w / 4, -h / 2, -l * 1.6);
  glVertex3f(-w / 2, -h / 2, -l);
  glVertex3f(-w / 4, -h, -l);
  glVertex3f(-w / 4, -h, -l * 1.6);

  //lower  slope window
  glVertex3f(w / 4, -h, -l * 1.6);
  glVertex3f(w / 4, -h, -l);
  glVertex3f(-w / 4, -h, -l);
  glVertex3f(-w / 4, -h, -l * 1.6);

  /////////////////////////////////////////////////////////////
  glColor3f(0.9f, 0.9f, 0.9f);

  //lower center right slope window
  glVertex3f(w / 2, -h / 2, -l);
  glVertex3f(w / 4, -h / 2, -l * 1.6);
  glVertex3f(w / 4, h / 4, -l * 1.4);
  glVertex3f(w / 2, h / 4, -l);

  //lower center left slope window
  glVertex3f(-w / 2, -h / 2, -l);
  glVertex3f(-w / 4, -h / 2, -l * 1.6);
  glVertex3f(-w / 4, h / 4, -l * 1.4);
  glVertex3f(-w / 2, h / 4, -l);

  //center center slope window
  glVertex3f(w / 4, -0.2f, -l * 2);
  glVertex3f(w / 4, h / 4, -l * 1.4);
  glVertex3f(-w / 4, h / 4, -l * 1.4);
  glVertex3f(-w / 4, -0.2f, -l * 2);

  //center center right slope window
  glVertex3f(w / 4, h / 4, -l * 1.4);
  glVertex3f(w / 4, -h / 2, -l * 1.6);
  glVertex3f(w / 4, -0.2f, -l * 2);
  glVertex3f(w / 4, h / 4, -l * 1.4);

  //center center right slope window
  glVertex3f(-w / 4, h / 4, -l * 1.4);
  glVertex3f(-w / 4, -h / 2, -l * 1.6);
  glVertex3f(-w / 4, -0.2f, -l * 2);
  glVertex3f(-w / 4, h / 4, -l * 1.4);

  glColor3f(0.8f, 0.0f, 0.2f);

  //lower center slope window
  glVertex3f(w / 4, -0.2f, -l * 2);
  glVertex3f(-w / 4, -0.2f, -l * 2);
  glVertex3f(-w / 4, -h, -l * 1.6);
  glVertex3f(w / 4, -h, -l * 1.6);

  //center center lower right slope window
  glVertex3f(w / 4, h / 4, -l * 1.4);
  glVertex3f(w / 4, -h, -l * 1.6);
  glVertex3f(w / 4, -0.2f, -l * 2);
  glVertex3f(w / 4, h / 4, -l * 1.4);

  //center center lower right slope window
  glVertex3f(-w / 4, h / 4, -l * 1.4);
  glVertex3f(-w / 4, -h, -l * 1.6);
  glVertex3f(-w / 4, -0.2f, -l * 2);
  glVertex3f(-w / 4, h / 4, -l * 1.4);

  ////////////////////////////Wing//////////////////////////////
  glColor3f(0.8f, 0.8f, 0.8f);

  //Right
  glVertex3f(w * 3, h, -1.5);
  glVertex3f(0.75f, h, -l);
  glVertex3f(0.75f, h, -0.5f);
  glVertex3f(w * 3, h, -0.7f);

  glVertex3f(w * 3, h + 0.01, -1.5);
  glVertex3f(0.75f, h + 0.01, -l);
  glVertex3f(0.75f, h + 0.01, -0.5f);
  glVertex3f(w * 3, h + 0.01, -0.7f);

  glVertex3f(w * 3, h + 0.02, -1.5);
  glVertex3f(0.75f, h + 0.02, -l);
  glVertex3f(0.75f, h + 0.02, -0.5f);
  glVertex3f(w * 3, h + 0.02, -0.7f);

  glVertex3f(w * 3, h + 0.03, -1.5);
  glVertex3f(0.75f, h + 0.03, -l);
  glVertex3f(0.75f, h + 0.03, -0.5f);
  glVertex3f(w * 3, h + 0.03, -0.7f);

  glVertex3f(w * 3, h - 0.01, -1.5);
  glVertex3f(0.75f, h - 0.01, -l);
  glVertex3f(0.75f, h - 0.01, -0.5f);
  glVertex3f(w * 3, h - 0.01, -0.7f);

  glVertex3f(w * 3, h - 0.02, -1.5);
  glVertex3f(0.75f, h - 0.02, -l);
  glVertex3f(0.75f, h - 0.02, -0.5f);
  glVertex3f(w * 3, h - 0.02, -0.7f);

  glVertex3f(w * 3, h - 0.03, -1.5);
  glVertex3f(0.75f, h - 0.03, -l);
  glVertex3f(0.75f, h - 0.03, -0.5f);
  glVertex3f(w * 3, h - 0.03, -0.7f);

  //Left
  glVertex3f(-w * 3, h, -1.5);
  glVertex3f(-0.75f, h, -l);
  glVertex3f(-0.75f, h, -0.5f);
  glVertex3f(-w * 3, h, -0.7f);

  glVertex3f(-w * 3, h + 0.01, -1.5);
  glVertex3f(-0.75f, h + 0.01, -l);
  glVertex3f(-0.75f, h + 0.01, -0.5f);
  glVertex3f(-w * 3, h + 0.01, -0.7f);

  glVertex3f(-w * 3, h + 0.02, -1.5);
  glVertex3f(-0.75f, h + 0.02, -l);
  glVertex3f(-0.75f, h + 0.02, -0.5f);
  glVertex3f(-w * 3, h + 0.02, -0.7f);

  glVertex3f(-w * 3, h + 0.03, -1.5);
  glVertex3f(-0.75f, h + 0.03, -l);
  glVertex3f(-0.75f, h + 0.03, -0.5f);
  glVertex3f(-w * 3, h + 0.03, -0.7f);

  glVertex3f(-w * 3, h - 0.01, -1.5);
  glVertex3f(-0.75f, h - 0.01, -l);
  glVertex3f(-0.75f, h - 0.01, -0.5f);
  glVertex3f(-w * 3, h - 0.01, -0.7f);

  glVertex3f(-w * 3, h - 0.02, -1.5);
  glVertex3f(-0.75f, h - 0.02, -l);
  glVertex3f(-0.75f, h - 0.02, -0.5f);
  glVertex3f(-w * 3, h - 0.02, -0.7f);

  glVertex3f(-w * 3, h - 0.03, -1.5);
  glVertex3f(-0.75f, h - 0.03, -l);
  glVertex3f(-0.75f, h - 0.03, -0.5f);
  glVertex3f(-w * 3, h - 0.03, -0.7f);

  ////////////////////////////////////////TailWing///////////////////////////////////////////////
  glColor3f(0.8f, 0.0f, 0.2f);

  //Top
  glVertex3f(0, h * 2.2, l * 1.5);
  glVertex3f(0, h * 2.0, l);
  glVertex3f(0, h / 2, l * 0.9);
  glVertex3f(0, h / 4, l * 1.5);

  glVertex3f(0.01, h * 2.2, l * 1.5);
  glVertex3f(0.01, h * 2.0, l);
  glVertex3f(0.01, h / 2, l * 0.9);
  glVertex3f(0.01, h / 4, l * 1.5);

  glVertex3f(0.02, h * 2.2, l * 1.5);
  glVertex3f(0.02, h * 2.0, l);
  glVertex3f(0.02, h / 2, l * 0.9);
  glVertex3f(0.02, h / 4, l * 1.5);

  glVertex3f(-0.01, h * 2.2, l * 1.5);
  glVertex3f(-0.01, h * 2.0, l);
  glVertex3f(-0.01, h / 2, l * 0.9);
  glVertex3f(-0.01, h / 4, l * 1.5);

  glVertex3f(-0.02, h * 2.2, l * 1.5);
  glVertex3f(-0.02, h * 2.0, l);
  glVertex3f(-0.02, h / 2, l * 0.9);
  glVertex3f(-0.02, h / 4, l * 1.5);

  //////////////////////////////////////Right////////////////////////////////////
  glVertex3f(w, h / 4, l * 1.5);
  glVertex3f(w, h / 4, l);
  glVertex3f(0, h / 4, l * 0.9);
  glVertex3f(0, h / 4, l * 1.5);

  glVertex3f(w, (h / 4) + 0.01, l * 1.5);
  glVertex3f(w, (h / 4) + 0.01, l);
  glVertex3f(0, (h / 4) + 0.01, l * 0.9);
  glVertex3f(0, (h / 4) + 0.01, l * 1.5);

  glVertex3f(w, (h / 4) + 0.02, l * 1.5);
  glVertex3f(w, (h / 4) + 0.02, l);
  glVertex3f(0, (h / 4) + 0.02, l * 0.9);
  glVertex3f(0, (h / 4) + 0.02, l * 1.5);

  glVertex3f(w, (h / 4) - 0.01, l * 1.5);
  glVertex3f(w, (h / 4) - 0.01, l);
  glVertex3f(0, (h / 4) - 0.01, l * 0.9);
  glVertex3f(0, (h / 4) - 0.01, l * 1.5);

  glVertex3f(w, (h / 4) - 0.02, l * 1.5);
  glVertex3f(w, (h / 4) - 0.02, l);
  glVertex3f(0, (h / 4) - 0.02, l * 0.9);
  glVertex3f(0, (h / 4) - 0.02, l * 1.5);

  //////////////////////////////////////Left////////////////////////////////////
  glVertex3f(-w, h / 4, l * 1.5);
  glVertex3f(-w, h / 4, l);
  glVertex3f(0, h / 4, l * 0.9);
  glVertex3f(0, h / 4, l * 1.5);

  glVertex3f(-w, (h / 4) + 0.01, l * 1.5);
  glVertex3f(-w, (h / 4) + 0.01, l);
  glVertex3f(0, (h / 4) + 0.01, l * 0.9);
  glVertex3f(0, (h / 4) + 0.01, l * 1.5);

  glVertex3f(-w, (h / 4) + 0.02, l * 1.5);
  glVertex3f(-w, (h / 4) + 0.02, l);
  glVertex3f(0, (h / 4) + 0.02, l * 0.9);
  glVertex3f(0, (h / 4) + 0.02, l * 1.5);

  glVertex3f(-w, (h / 4) - 0.01, l * 1.5);
  glVertex3f(-w, (h / 4) - 0.01, l);
  glVertex3f(0, (h / 4) - 0.01, l * 0.9);
  glVertex3f(0, (h / 4) - 0.01, l * 1.5);

  glVertex3f(-w, (h / 4) - 0.02, l * 1.5);
  glVertex3f(-w, (h / 4) - 0.02, l);
  glVertex3f(0, (h / 4) - 0.02, l * 0.9);
  glVertex3f(0, (h / 4) - 0.02, l * 1.5);

  glEnd();
  //////////////////////////////////////////////////////////////////////////////////////////////

  glBegin(GL_TRIANGLES);
  ////////////////////////////////////////Tail///////////////////////////////////////////////
  glColor3f(0.9f, 0.9f, 0.9f);
  //top
  glVertex3f(w / 4, h, l / 2);
  glVertex3f(-w / 4, h, l / 2);
  glVertex3f((-w / 4 + w / 4) / 2, h / 4, l * 1.5);

  //right
  glVertex3f(0, 0.38, l * 1.5);
  glVertex3f(w / 2, h / 2, l / 2);
  glVertex3f(w / 2, -h / 2, l / 2);

  //upper right
  glVertex3f(0, h / 4, l * 1.5);
  glVertex3f(w / 4, h, l / 2);
  glVertex3f(w / 2, h / 2, l / 2);

  // //left
  glVertex3f(0, 0.38, l * 1.5);
  glVertex3f(-w / 2, h / 2, l / 2);
  glVertex3f(-w / 2, -h / 2, l / 2);

  // //upper left
  glVertex3f(0, h / 4, l * 1.5);
  glVertex3f(-w / 4, h, l / 2);
  glVertex3f(-w / 2, h / 2, l / 2);

  glColor3f(0.8f, 0.0f, 0.2f);
  //lower right
  glVertex3f(0, h / 3.9, l * 1.5);
  glVertex3f(w / 4, -h, l / 2);
  glVertex3f(w / 2, -h / 2, l / 2);

  // //lower
  glVertex3f(w / 4, -h, l / 2);
  glVertex3f(-w / 4, -h, l / 2);
  glVertex3f((-w / 4 + w / 4) / 2, h / 4, l * 1.5);

  // //lower left
  glVertex3f(0, h / 3.9, l * 1.5);
  glVertex3f(-w / 4, -h, l / 2);
  glVertex3f(-w / 2, -h / 2, l / 2);

  glEnd();

  glBegin(GL_TRIANGLES);
  glColor3f(0.9f, 0.2f, 0.2f);
  ///////////////////////////////Top//////////////////////////////////////////////////////////
  glVertex3f(0.15, -0.2, -l * 2);
  glVertex3f(-0.15, -0.2, -l * 2);
  glVertex3f(((0.15) + (-0.15)) / 2, -0.3, -l * 2.15);

  ///////////////////////////////RightTop//////////////////////////////////////////////////////////
  glVertex3f(-0.15, -0.2, -l * 2);
  glVertex3f(-0.3, -0.3, -l * 2);
  glVertex3f(((0.15) + (-0.15)) / 2, -0.3, -l * 2.15);

  // ///////////////////////////////Right//////////////////////////////////////////////////////////
  glVertex3f(-0.3, -0.3, -l * 2);
  glVertex3f(-0.3, -0.4, -l * 2);
  glVertex3f(((0.15) + (-0.15)) / 2, -0.3, -l * 2.15);

  // ///////////////////////////////lowerRight//////////////////////////////////////////////////////////
  glVertex3f(-0.3, -0.4, -l * 2);
  glVertex3f(-0.15, -0.5, -l * 2);
  glVertex3f(((0.15) + (-0.15)) / 2, -0.3, -l * 2.15);

  // ///////////////////////////////Lower//////////////////////////////////////////////////////////
  glVertex3f(-0.15, -0.5, -l * 2);
  glVertex3f(0.15, -0.5, -l * 2);
  glVertex3f(((0.15) + (-0.15)) / 2, -0.3, -l * 2.15);

  // ///////////////////////////////LeftLower//////////////////////////////////////////////////////////
  glVertex3f(0.15, -0.5, -l * 2);
  glVertex3f(0.3, -0.4, -l * 2);
  glVertex3f(((0.15) + (-0.15)) / 2, -0.3, -l * 2.15);

  // // ///////////////////////////////Left//////////////////////////////////////////////////////////
  glVertex3f(0.3, -0.4, -l * 2);
  glVertex3f(0.3, -0.3, -l * 2);
  glVertex3f(((0.15) + (-0.15)) / 2, -0.3, -l * 2.15);

  // ///////////////////////////////TopLeft//////////////////////////////////////////////////////////
  glVertex3f(0.3, -0.3, -l * 2);
  glVertex3f(0.15, -0.2, -l * 2);
  glVertex3f(((0.15) + (-0.15)) / 2, -0.3, -l * 2.15);
  glEnd();

  glPopMatrix();
}

/*____________________________________________________________________
|
| Function: Draw_Particles
|
| Input:
| Output: Draw particles as view-oriented billboards.
|___________________________________________________________________*/

void Draw_Particles()
{
  int i;
  gmtl::Matrix44f cam_mat;                // Camera matrix
  gmtl::Matrix44f dt_mat, el_mat, az_mat; // distance, elevation, and azimuth matrices, initialized to IDENTITY
  float age_scale;                        // Age factor

  /*____________________________________________________________________
|
| Enable texturing and blending
|___________________________________________________________________*/

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture);
  glDepthMask(GL_FALSE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE);

  /*____________________________________________________________________
|
| Orient billboards to face the camera
|___________________________________________________________________*/

  // Set distance matrix
  dt_mat[2][3] = distance[cam_id];

  // Set elevation matrix
  gmtl::set(el_mat,
            gmtl::AxisAnglef(gmtl::Math::deg2Rad(elevation[cam_id]), 1, 0, 0));

  // Set azimuth matrix
  gmtl::set(az_mat,
            gmtl::AxisAnglef(gmtl::Math::deg2Rad(azimuth[cam_id]), 0, 1, 0));

  // Compute camera w.r.t. world and discard translation
  if (cam_id == 0)
  {
    cam_mat = az_mat * el_mat * dt_mat;
  }
  else if (cam_id == 1)
  {
    cam_mat = pposeadj * az_mat * el_mat * dt_mat;
  }
  else if (cam_id == 2)
  {
    cam_mat = pposeadj2 * az_mat * el_mat * dt_mat;
  }
  cam_mat[0][3] = cam_mat[1][3] = cam_mat[2][3] = 0;

  /*____________________________________________________________________
|
| Render particles as billboards
|___________________________________________________________________*/

  for (i = 0; i < PARTICLE_NB; i++)
  {
    glPushMatrix();
    glTranslatef(particles[i].p[0], particles[i].p[1], particles[i].p[2]);
    glMultMatrixf(cam_mat.mData); // Orient billboards to face camera
    glBegin(GL_QUADS);
    age_scale = ((float)particles[i].ttl) / particles[i].ttl_init;
    glColor3f(age_scale, age_scale, age_scale);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-SMOKE_SIZE, -SMOKE_SIZE, 0.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(SMOKE_SIZE, -SMOKE_SIZE, 0.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(SMOKE_SIZE, SMOKE_SIZE, 0.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-SMOKE_SIZE, SMOKE_SIZE, 0.0);
    glEnd();
    glPopMatrix();
  }
  glDisable(GL_BLEND);
  glDepthMask(GL_TRUE);
  glDisable(GL_TEXTURE_2D);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture2);
  glDepthMask(GL_FALSE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE);
  for (i = 0; i < PARTICLE_NB; i++)
  {
    glPushMatrix();
    glTranslatef(particles2[i].p[0], particles2[i].p[1], particles2[i].p[2]);
    glMultMatrixf(cam_mat.mData); // Orient billboards to face camera
    glBegin(GL_QUADS);
    age_scale = ((float)particles2[i].ttl) / particles2[i].ttl_init;
    glColor3f(age_scale, age_scale, age_scale);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-SMOKE_SIZE, -SMOKE_SIZE, 0.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(SMOKE_SIZE, -SMOKE_SIZE, 0.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(SMOKE_SIZE, SMOKE_SIZE, 0.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-SMOKE_SIZE, SMOKE_SIZE, 0.0);
    glEnd();
    glPopMatrix();
  }
  /*____________________________________________________________________
|
| Restore rendering states
|___________________________________________________________________*/

  glDisable(GL_BLEND);
  glDepthMask(GL_TRUE);
  glDisable(GL_TEXTURE_2D);
}

/*
	Random number generator by Donald H. House.
	Modified to compile on Visual Studio.Net 2003
*/
float FastGauss(float mean, float std)
{
#define RESOLUTION 2500
  static float lookup[RESOLUTION + 1];

#define itblmax 20
/* length - 1 of table describing F inverse */
#define didu 40.0
  /* delta table position / delta ind. variable           itblmax / 0.5 */

  static float tbl[] =
      {0.00000E+00, 6.27500E-02, 1.25641E-01, 1.89000E-01,
       2.53333E-01, 3.18684E-01, 3.85405E-01, 4.53889E-01,
       5.24412E-01, 5.97647E-01, 6.74375E-01, 7.55333E-01,
       8.41482E-01, 9.34615E-01, 1.03652E+00, 1.15048E+00,
       1.28167E+00, 1.43933E+00, 1.64500E+00, 1.96000E+00,
       3.87000E+00};

  static int hereb4;
  //static struct timeval tv;

  float u, di, delta /*, result*/;
  int i, index, minus = 1;

  if (!hereb4)
  {
    for (i = 0; i <= RESOLUTION; i++)
    {
      if ((u = i / (float)RESOLUTION) > 0.5)
      {
        minus = 0;
        u -= 0.5;
      }

      /* interpolate gaussian random number using table */

      index = (int)(di = (didu * u));
      di -= (float)index;
      delta = tbl[index] + (tbl[index + 1] - tbl[index]) * di;
      lookup[i] = (minus ? -delta : delta);
    }

    /*gettimeofday(&tv, NULL);
		srand((unsigned int)tv.tv_usec);*/
    srand((unsigned)time(NULL));
    hereb4 = 1;
  }

  i = rand() / (RAND_MAX / RESOLUTION) + 1;
  if (i > RESOLUTION)
  {
    i = RESOLUTION;
  }

  return (mean + std * lookup[i]);
}

/*____________________________________________________________________
|
| Function: LoadPPM
|
| Input:
| Output: Draw world axes.
|  LoadPPM - a minimal Portable Pixelformat image file loader
|  fname: name of file to load (input)
|  w: width of loaded image in pixels (output)
|  h: height of loaded image in pixels (output)
|  data: image data address (input or output depending on mallocflag)
|  mallocflag: 1 if memory not pre-allocated, 0 if data already points
|              to allocated memory that can hold the image. Note that 
|              if new memory is allocated, free() should be used to 
|              deallocate when it is no longer needed.
|___________________________________________________________________*/

void LoadPPM(char *fname, unsigned int *w, unsigned int *h, unsigned char **data, int mallocflag)
{
  FILE *fp;
  char P, num;
  int max;
  char s[1000];

  if (!(fp = fopen(fname, "rb")))
  {
    perror("cannot open image file\n");
    exit(0);
  }

  fscanf(fp, "%c%c\n", &P, &num);
  if ((P != 'P') || (num != '6'))
  {
    perror("unknown file format for image\n");
    exit(0);
  }

  do
  {
    fgets(s, 999, fp);
  } while (s[0] == '#');

  sscanf(s, "%d%d", w, h);
  fgets(s, 999, fp);
  sscanf(s, "%d", &max);

  if (mallocflag)
    if (!(*data = (unsigned char *)malloc(*w * *h * 3)))
    {
      perror("cannot allocate memory for image data\n");
      exit(0);
    }

  fread(*data, 3, *w * *h, fp);

  fclose(fp);
}

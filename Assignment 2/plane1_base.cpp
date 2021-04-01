//|___________________________________________________________________
//!
//! \file plane1_base.cpp
//!
//! \brief Base source code for the first plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard controls:
//!   s   = moves the plane forward
//!   f   = moves the plane backward
//!   q,e = rolls the plane
//!
//!   k   = moves the camera forward
//!   ;   = moves the camera backward
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

//|___________________
//|
//| Includes
//|___________________

#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>
#include <windows.h>
#include <iostream>
using namespace std;

//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH = 3;
const float P_LENGTH = 3;
const float P_HEIGHT = 1.5;

// Camera's view frustum
const float CAM_FOV = 60.0f; // Field of view in degs

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width = 800;
int w_height = 600;

// Plane pose (position & orientation)
gmtl::Matrix44f plane_pose; // T, as defined in the handout, initialized to IDENTITY by default

// Camera pose
gmtl::Matrix44f cam_pose; // C, as defined in the handout
gmtl::Matrix44f view_mat; // View transform is C^-1 (inverse of the camera transform C)

// Transformation matrices applied to plane and camera poses
gmtl::Matrix44f ztransp_mat;
gmtl::Matrix44f ztransn_mat;
gmtl::Matrix44f xtransp_mat;
gmtl::Matrix44f xtransn_mat;
gmtl::Matrix44f ytransp_mat;
gmtl::Matrix44f ytransn_mat;
gmtl::Matrix44f zrotp_mat;
gmtl::Matrix44f zrotn_mat;
gmtl::Matrix44f xrotp_mat;
gmtl::Matrix44f xrotn_mat;
gmtl::Matrix44f yrotp_mat;
gmtl::Matrix44f yrotn_mat;

//|___________________
//|
//| Function Prototypes
//|___________________

void InitMatrices();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
// void DrawPlane(const float width, const float length, const float height);
void DrawPlane2();
void SpecialKeys(int key, int x, int y);

//|____________________________________________________________________
//|
//| Function: InitMatrices
//|
//! \param None.
//! \return None.
//!
//! Initializes all the matrices
//|____________________________________________________________________

void InitMatrices()
{
  const float TRANS_AMOUNT = 1.0f;
  const float ROT_AMOUNT = gmtl::Math::deg2Rad(5.0f); // specified in degs, but get converted to radians

  const float COSTHETA = cos(ROT_AMOUNT);
  const float SINTHETA = sin(ROT_AMOUNT);

  // Positive Z-Translation
  ztransp_mat.set(1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, TRANS_AMOUNT,
                  0, 0, 0, 1);
  ztransp_mat.setState(gmtl::Matrix44f::TRANS);

  gmtl::invert(ztransn_mat, ztransp_mat);

  // Positive X-Translation
  xtransp_mat.set(1, 0, 0, TRANS_AMOUNT,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1);
  xtransp_mat.setState(gmtl::Matrix44f::TRANS);

  gmtl::invert(xtransn_mat, xtransp_mat);

  // Positive Y-Translation
  ytransp_mat.set(1, 0, 0, 0,
                  0, 1, 0, TRANS_AMOUNT,
                  0, 0, 1, 0,
                  0, 0, 0, 1);
  ytransp_mat.setState(gmtl::Matrix44f::TRANS);

  gmtl::invert(ytransn_mat, ytransp_mat);

  // Positive Z-rotation (roll)
  zrotp_mat.set(COSTHETA, -SINTHETA, 0, 0,
                SINTHETA, COSTHETA, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
  zrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

  // Negative Z-rotation (roll)
  gmtl::invert(zrotn_mat, zrotp_mat);

  // Positive X-rotation (picth)
  xrotp_mat.set(
      1, 0, 0, 0,
      0, COSTHETA, -SINTHETA, 0,
      0, SINTHETA, COSTHETA, 0,
      0, 0, 0, 1);
  xrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);
  // Negative X-rotation (roll)
  gmtl::invert(xrotn_mat, xrotp_mat);

  // Positive Y-rotation (yall)
  yrotp_mat.set(
      COSTHETA, 0, SINTHETA, 0,
      0, 1, 0, 0,
      -SINTHETA, 0, COSTHETA, 0,
      0, 0, 0, 1);
  yrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);
  // Negative Y-rotation (roll)
  gmtl::invert(yrotn_mat, yrotp_mat);

  // Inits plane pose
  plane_pose.set(1, 0, 0, 1.0f,
                 0, 1, 0, 0.0f,
                 0, 0, 1, 4.0f,
                 0, 0, 0, 1.0f);
  plane_pose.setState(gmtl::Matrix44f::AFFINE); // AFFINE because the plane pose can contain both translation and rotation

  // Inits camera pose and view transform
  cam_pose.set(1, 0, 0, 2.0f,
               0, 1, 0, 1.0f,
               0, 0, 1, 15.0f,
               0, 0, 0, 1.0f);
  cam_pose.setState(gmtl::Matrix44f::AFFINE);
  gmtl::invert(view_mat, cam_pose); // View transform is the inverse of the camera pose
}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
  glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
  // Modelview matrix
  gmtl::Matrix44f modelview_mat; // M, as defined in the handout

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //|____________________________________________________________________
  //|
  //| Viewport 1 rendering: shows the moving camera's view
  //|____________________________________________________________________

  glViewport(0, 0, (GLsizei)w_width / 2, (GLsizei)w_height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width / (2 * w_height), 0.1f, 100.0f); // Check MSDN: google "gluPerspective msdn"

  // Approach1
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); // A good practice for beginner

  // Draws world coordinate frame
  modelview_mat = view_mat; // M = C^-1
  glLoadMatrixf(modelview_mat.mData);
  DrawCoordinateFrame(10);

  // Draws plane and its local frame
  modelview_mat *= plane_pose; // M = C^-1 * T
  glLoadMatrixf(modelview_mat.mData);
  // DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawPlane2();
  DrawCoordinateFrame(3);

  /*
  // Approach 2 (gives the same results as the approach 1)
  glMatrixMode(GL_MODELVIEW);

  // Draws world coordinate frame
  glLoadMatrixf(view_mat.mData);             // M = C^-1
  DrawCoordinateFrame(10);

  // Draws plane and its local frame
  glMultMatrixf(plane_pose.mData);           // M = C^-1 * T (OpenGL calls build transforms in left-to-right order)
  DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);
*/

  //|____________________________________________________________________
  //|
  //| TODO: Viewport 2 rendering: shows the fixed top-down view
  //|____________________________________________________________________

  // glViewport...

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width / (2 * w_height), 0.1f, 100.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // ...

  glFlush();
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________
void SpecialKeys(int key, int x, int y)
{
  // cout << key;
  switch (key)
  {
  case GLUT_KEY_LEFT:
    plane_pose = plane_pose * xtransn_mat;
    break;
  case GLUT_KEY_RIGHT:
    plane_pose = plane_pose * xtransp_mat;
    break;
  case GLUT_KEY_UP:
    plane_pose = plane_pose * ztransn_mat;
    break;
  case GLUT_KEY_DOWN:
    plane_pose = plane_pose * ztransp_mat;
    break;
  }
  glutPostRedisplay();
}
void KeyboardFunc(unsigned char key, int x, int y)
{
  switch (key)
  {
    //|____________________________________________________________________
    //|
    //| Plane controls
    //|____________________________________________________________________

  case 'n': // Forward translation of the plane (positive Z-translation)
    plane_pose = plane_pose * ztransp_mat;
    break;
  case 'f': // Backward translation of the plane
    plane_pose = plane_pose * ztransn_mat;
    break;

  case 'q': // Rolls the plane (+ Z-rot)
    plane_pose = plane_pose * zrotp_mat;
    break;
  case 'e': // Rolls the plane (- Z-rot)
    plane_pose = plane_pose * zrotn_mat;
    break;
  case 'w': // Rolls the plane (+ X-rot)
    plane_pose = plane_pose * xrotp_mat;
    break;
  case 's': // Rolls the plane - X-rot)
    plane_pose = plane_pose * xrotn_mat;
    break;
  case 'a': // Rolls the plane (+ Y-rot)
    plane_pose = plane_pose * yrotp_mat;
    break;
  case 'd': // Rolls the plane (- Y-rot)
    plane_pose = plane_pose * yrotn_mat;
    break;

    // TODO: Add the remaining controls/transforms
    //|____________________________________________________________________
    //|
    //| Camera controls
    //|____________________________________________________________________

  case '8': // Forward translation of the camera (negative Z-translation - cameras looks in its (local) -Z direction)
    cam_pose = cam_pose * ztransn_mat;
    break;
  case '2': // Backward translation of the camera
    cam_pose = cam_pose * ztransp_mat;
    break;
    // TODO: Add the remaining controls
  case '4': // Forward translation of the camera (negative Z-translation - cameras looks in its (local) -Z direction)
    cam_pose = cam_pose * xtransn_mat;
    break;
  case '6': // Backward translation of the camera
    cam_pose = cam_pose * xtransp_mat;
    break;
  case '*': // Forward translation of the camera (negative Z-translation - cameras looks in its (local) -Z direction)
    cam_pose = cam_pose * ytransn_mat;
    break;
  case '/': // Backward translation of the camera
    cam_pose = cam_pose * ytransp_mat;
    break;
  default:
    glutSpecialFunc(SpecialKeys);
  }

  gmtl::invert(view_mat, cam_pose); // Updates view transform to reflect the change in camera transform
  glutPostRedisplay();              // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
  // Track the current window dimensions
  w_width = w;
  w_height = h;
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
  glBegin(GL_LINES);
  // X axis is red
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(l, 0.0f, 0.0f);

  // Y axis is green
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, l, 0.0f);

  // Z axis is blue
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, l);
  glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawPlane
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws the plane.
//|____________________________________________________________________

void DrawPlane(const float width, const float length, const float height)
{
  float w = width / 2;
  float l = length / 2;
  float h = height / 2;

  glBegin(GL_QUADS);
  glColor3f(1.0f, 0.0f, 0.0f);
  //front
  glVertex3f(w / 2, h, l);
  glVertex3f(-w / 2, h, l);
  glVertex3f(-w / 2, -h, l);
  glVertex3f(w / 2, -h, l);
  //back
  glVertex3f(w / 2, h, -l);
  glVertex3f(w / 2, -h, -l);
  glVertex3f(-w / 2, -h, -l);
  glVertex3f(-w / 2, h, -l);
  //Right
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(w / 2, h, -l);
  glVertex3f(w / 2, h, l);
  glVertex3f(w / 2, -h, l);
  glVertex3f(w / 2, -h, -l);

  //LEFT
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(-w / 2, h, -l);
  glVertex3f(-w / 2, h, l);
  glVertex3f(-w / 2, -h, l);
  glVertex3f(-w / 2, -h, -l);

  //Top
  glColor3f(1.0f, 0.0f, 1.0f);
  glVertex3f(w / 2, h, -l);
  glVertex3f(-w / 2, h, -l);
  glVertex3f(-w / 2, h, l);
  glVertex3f(w / 2, h, l);

  //Bottom
  glColor3f(1.0f, 0.0f, 1.0f);
  glVertex3f(w / 2, -h, -l);
  glVertex3f(-w / 2, -h, -l);
  glVertex3f(-w / 2, -h, l);
  glVertex3f(w / 2, -h, l);

  glEnd();

  glBegin(GL_TRIANGLES);
  // Body is red
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, l);
  glVertex3f(w * 2, 0.0f, -l);
  glVertex3f(-w * 2, 0.0f, -l);

  // Wing is blue
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, -l);
  glVertex3f(0.0f, height * 1.5, -l);
  glEnd();
}

void DrawPlane2()
{
  const float w = 3.0f;
  const float h = 1.5f;
  const float l = 3.0f;

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
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(w / 4, -0.2f, -l * 2);
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(w / 4, h / 4, -l * 1.4);
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(-w / 4, h / 4, -l * 1.4);
  glColor3f(1.0f, 1.0f, 1.0f);
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
  //////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////Propeller ////////////////////////////////////////////

  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0.1, -0.2f, -l * 2.1);
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0.1, -0.1f, -l * 1.8);
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(-0.1, -0.1f, -l * 1.8);

  /////////////////////////////////////////////////////////////////////////////////////////////
  glEnd();
}
//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char **argv)
{
  InitMatrices();

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(w_width, w_height);

  glutCreateWindow("Plane Episode 1");

  glutDisplayFunc(DisplayFunc);
  glutReshapeFunc(ReshapeFunc);
  glutKeyboardFunc(KeyboardFunc);

  InitGL();

  glutMainLoop();

  return 0;
}
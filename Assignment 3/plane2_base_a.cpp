//|___________________________________________________________________
//!
//! \file plane2_base.cpp
//!
//! \brief Base source code for the second plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard inputs for plane and propeller (subpart):
//!   s   = moves the plane forward
//!   f   = moves the plane backward
//!   q,e = rolls the plane
//!   a   = yaws the plane
//!   x   = pitches the plane
//!
//!   r   = rotates propeller
//!
//! Mouse inputs for world-relative camera:
//!   Hold left button and drag  = controls azimuth and elevation
//!                                (Press CTRL (and hold) before left button to restrict to azimuth control only,
//!                                 Press SHIFT (and hold) before left button to restrict to elevation control only)
//!   Hold right button and drag = controls distance
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

//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH = 3;
const float P_LENGTH = 3;
const float P_HEIGHT = 1.5f;

// Propeller rotation (subpart)
float pp_angle = 90; // Rotation angle

// Plane transforms
const gmtl::Vec3f PLANE_FORWARD(0, 0, 1.0f); // Plane's forward translation vector (w.r.t. local frame)
const float PLANE_ROTATION = 10.0f;          // Plane rotated by 5 degs per input

// Propeller dimensions (subpart)
const float PP_WIDTH = 0.25f;
const float PP_LENGTH = 1.5f;

// Propeller transforms
const gmtl::Point3f PROPELLER_POS(0, -0.3, (-P_LENGTH / 2) * 3.1); // Propeller position on the plane (w.r.t. plane's frame)
const float PROPELLER_ROTATION = 10.0f;                            // Propeller rotated by 5 degs per input
const gmtl::Point3f PROPELLER_POS1(20.0f, 9.8f, 7.5f);
// Camera's view frustum
const float CAM_FOV = 90.0f; // Field of view in degs

// Keyboard modifiers
enum KeyModifier
{
  KM_SHIFT = 0,
  KM_CTRL,
  KM_ALT
};

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width = 800;
int w_height = 600;

// Plane pose (position-quaternion pair)
gmtl::Point4f plane_p, plane1Position, ppPosition;   // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q, plane1Quaternion, ppQuaternion; // Quaternion

// Quaternions to rotate plane
gmtl::Quatf zrotp_q, xrotp_q, yrotp_q, ppzrotp_q; // Positive and negative Z rotations
gmtl::Quatf zrotn_q, xrotn_q, yrotn_q, ppzrotn_q;

// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3] = {false, false, false};
bool kmodifiers[3] = {false, false, false};

// Cameras
int cam_id = 0;                        // Selects which camera to view
int camctrl_id = 0;                    // Selects which camera to control
float distance[2] = {20.0f, 20.0f};    // Distance of the camera from world's origin.
float elevation[2] = {-45.0f, -45.0f}; // Elevation of the camera. (in degs)
float azimuth[2] = {15.0f, 15.0f};     // Azimuth of the camera. (in degs)
int plane_id = 0;
//|___________________
//|
//| Function Prototypes
//|___________________

void InitTransforms();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawPlaneBody(const float width, const float length, const float height);
void DrawPropeller(const float width, const float length);

//|____________________________________________________________________
//|
//| Function: InitTransforms
//|
//! \param None.
//! \return None.
//!
//! Initializes all the transforms
//|____________________________________________________________________

void InitTransforms()
{
  const float COSTHETA_D2 = cos(gmtl::Math::deg2Rad(PLANE_ROTATION / 2)); // cos() and sin() expect radians
  const float SINTHETA_D2 = sin(gmtl::Math::deg2Rad(PLANE_ROTATION / 2));

  // Inits plane pose
  plane_p.set(1.0f, 0.0f, 4.0f, 1.0f);
  plane_q.set(0, 0, 0, 1);

  plane1Position.set(20.0f, 10.0f, 12.0f, 2.0f);
  plane1Quaternion.set(0, 0, 0, 1);
  ppQuaternion.set(0, 0, 0, 1);
  // Z rotations (roll)
  zrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2); // +Z
  zrotn_q = gmtl::makeConj(zrotp_q);           // -Z
  // X rotation (pitch)
  xrotp_q.set(SINTHETA_D2, 0, 0, COSTHETA_D2); // +X
  xrotn_q = gmtl::makeConj(xrotp_q);
  // Y rotation (yaw)
  yrotp_q.set(0, SINTHETA_D2, 0, COSTHETA_D2); // +Y
  yrotn_q = gmtl::makeConj(yrotp_q);
  // TODO: Initializes the remaining transforms

  ppzrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2);
  ppzrotn_q = gmtl::makeConj(zrotp_q);
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
  gmtl::AxisAnglef aa; // Converts plane's quaternion to axis-angle form to be used by glRotatef()
  gmtl::Vec3f axis;    // Axis component of axis-angle representation
  float angle;         // Angle component of axis-angle representation

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width / w_height, 0.1f, 1000.0f); // Check MSDN: google "gluPerspective msdn"

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  //|____________________________________________________________________
  //|
  //| Setting up view transform by:
  //| "move up to the world frame by composing all of the (inverse) transforms from the camera up to the world node"
  //|____________________________________________________________________

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

    gmtl::set(aa, plane_q); // Converts plane's quaternion to axis-angle form to be used by glRotatef()
    axis = aa.getAxis();
    angle = aa.getAngle();
    glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
    glTranslatef(-plane_p[0], -plane_p[1], -plane_p[2]);
    break;
  case 2:
    // For plane1's camera
    glTranslatef(0, 0, -distance[2]);
    glRotatef(-elevation[2], 1, 0, 0);
    glRotatef(-azimuth[2], 0, 1, 0);

    gmtl::set(aa, plane1Quaternion); // Converts plane's quaternion to axis-angle form to be used by glRotatef()
    axis = aa.getAxis();
    angle = aa.getAngle();
    glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
    glTranslatef(-plane1Position[0], -plane1Position[1], -plane1Position[2]);
    break;

    // TODO: Add case for the plane1's camera
  }

  //|____________________________________________________________________
  //|
  //| Draw traversal begins, start from world (root) node
  //|____________________________________________________________________

  // World node: draws world coordinate frame
  DrawCoordinateFrame(10);

  // World-relative camera:
  if (cam_id != 0)
  {
    glPushMatrix();
    glRotatef(azimuth[0], 0, 1, 0);
    glRotatef(elevation[0], 1, 0, 0);
    glTranslatef(0, 0, distance[0]);
    DrawCoordinateFrame(1);
    glPopMatrix();
  }

  // Plane 2 body:
  glPushMatrix();
  gmtl::set(aa, plane_q); // Converts plane's quaternion to axis-angle form to be used by glRotatef()
  axis = aa.getAxis();
  angle = aa.getAngle();
  glTranslatef(plane_p[0], plane_p[1], plane_p[2]);
  glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
  DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);

  // Plane 2's camera:
  glPushMatrix();
  glRotatef(azimuth[1], 0, 1, 0);
  glRotatef(elevation[1], 1, 0, 0);
  glTranslatef(0, 0, distance[1]);
  DrawCoordinateFrame(1);
  glPopMatrix();

  // Propeller (subpart):
  glPushMatrix();
  glTranslatef(PROPELLER_POS[0], PROPELLER_POS[1], PROPELLER_POS[2]); // Positions propeller on the plane
  glRotatef(pp_angle, 0, 0, 1);                                       // Rotates propeller
  DrawPropeller(PP_WIDTH, PP_LENGTH);
  DrawCoordinateFrame(1);
  glPopMatrix();
  glPopMatrix();

  // Propeller (subpart):
  glPushMatrix();
  glTranslatef(PROPELLER_POS1[0], PROPELLER_POS1[1], PROPELLER_POS1[2]); // Positions propeller on the plane
  glRotatef(pp_angle, 0, 0, 1);                                          // Rotates propeller
  DrawPropeller(PP_WIDTH, PP_LENGTH);
  DrawCoordinateFrame(1);
  glPopMatrix();
  glPopMatrix();

  // Plane 1 body:
  glPushMatrix();
  gmtl::set(aa, plane1Quaternion); // Converts plane's quaternion to axis-angle form to be used by glRotatef()
  axis = aa.getAxis();
  angle = aa.getAngle();
  glTranslatef(plane1Position[0], plane1Position[1], plane1Position[2]);
  glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
  DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);

  // Plane 1's camera:
  glPushMatrix();
  glRotatef(azimuth[2], 0, 1, 0);
  glRotatef(elevation[2], 1, 0, 0);
  glTranslatef(0, 0, distance[2]);
  DrawCoordinateFrame(1);
  glPopMatrix();
  glPopMatrix();

  glutSwapBuffers(); // Replaces glFlush() to use double buffering
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param key    [in] Key code.
//! \param x      [in] X-coordinate of mouse when key is pressed.
//! \param y      [in] Y-coordinate of mouse when key is pressed.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
  switch (key)
  {
    //|____________________________________________________________________
    //|
    //| Camera switch
    //|____________________________________________________________________

  case 'v': // Select camera to view
    cam_id = (cam_id + 1) % 3;
    printf("View camera = %d\n", cam_id);
    break;
  case 'b': // Select camera to control
    camctrl_id = (camctrl_id + 1) % 3;
    printf("Control camera = %d\n", camctrl_id);
    break;

  //|____________________________________________________________________
  //|
  //| Plane controls
  //|____________________________________________________________________
  case 'n':
    plane_id = (plane_id + 1) % 2;
    printf("Active plane = %d\n", plane_id + 1);
    break;
  case 'c':
  { // Forward translation of the plane (+Z translation)
    if (plane_id == 1)
    {
      gmtl::Quatf v_q = plane1Quaternion * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane1Quaternion);
      plane1Position = plane1Position + v_q.mData;
    }
    else
    {
      gmtl::Quatf v_q = plane_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
      plane_p = plane_p + v_q.mData;
    }
  }
  break;
  case ' ':
  { // Backward translation of the plane (-Z translation)
    if (plane_id == 1)
    {
      gmtl::Quatf v_q = plane1Quaternion * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane1Quaternion);
      plane1Position = plane1Position + v_q.mData;
    }
    else
    {
      gmtl::Quatf v_q = plane_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
      plane_p = plane_p + v_q.mData;
    }
  }
  break;

  case 'q': // Rolls the plane (+Z rot)
    if (plane_id == 1)
    {
      plane1Quaternion = plane1Quaternion * zrotp_q;
    }
    else
    {
      plane_q = plane_q * zrotp_q;
    }
    break;
  case 'e': // Rolls the plane (-Z rot)
    if (plane_id == 1)
    {
      plane1Quaternion = plane1Quaternion * zrotn_q;
    }
    else
    {
      plane_q = plane_q * zrotn_q;
    }
    break;

  case 'w': // Pitches the plane (+X rot)
    if (plane_id == 1)
    {
      plane1Quaternion = plane1Quaternion * xrotp_q;
    }
    else
    {
      plane_q = plane_q * xrotp_q;
    }
    break;
  case 's': // Pitches the plane (-X rot)
    if (plane_id == 1)
    {
      plane1Quaternion = plane1Quaternion * xrotn_q;
    }
    else
    {
      plane_q = plane_q * xrotn_q;
    }
    break;
  case 'a': // Yaws the plane (+Y rot)
    if (plane_id == 1)
    {
      plane1Quaternion = plane1Quaternion * yrotp_q;
    }
    else
    {
      plane_q = plane_q * yrotp_q;
    }
    break;
  case 'd': // Yaws the plane (-Y rot)
    if (plane_id == 1)
    {
      plane1Quaternion = plane1Quaternion * yrotn_q;
    }
    else
    {
      plane_q = plane_q * yrotn_q;
    }
    break;

    //|____________________________________________________________________
    //|
    //| Propeller controls (axis)
    //|____________________________________________________________________

  case 'r': // Rotates propeller Y axis
    pp_angle += PROPELLER_ROTATION;
    break;
  }
  glutPostRedisplay(); // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MouseFunc
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void MouseFunc(int button, int state, int x, int y)
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
//| Function: MotionFunc
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void MotionFunc(int x, int y)
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
        elevation[camctrl_id] += dy; // Elevation update
      }
      if (!kmodifiers[KM_SHIFT])
      {
        azimuth[camctrl_id] += dx; // Azimuth update
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
      distance[camctrl_id] += d;
    }

    glutPostRedisplay(); // Asks GLUT to redraw the screen
  }
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
  glViewport(0, 0, (GLsizei)w_width, (GLsizei)w_height);
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
//| Function: DrawPlaneBody
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws a plane body.
//|____________________________________________________________________

void DrawPlaneBody(const float width, const float length, const float height)
{
  const float w = width;
  const float h = height;
  const float l = length;

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
}

//|____________________________________________________________________
//|
//| Function: DrawPropeller
//|
//! \param width       [in] Width  of the propeller.
//! \param length      [in] Length of the propeller.
//! \return None.
//!
//! Draws a propeller.
//|____________________________________________________________________

void DrawPropeller(const float width, const float length)
{

  float w = width / 2;
  float l = length / 2;

  glBegin(GL_QUADS);
  // Propeller is white
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex3f(-w, -l, -l * 2);
  glVertex3f(w, -l, -l * 2);
  glVertex3f(w, l, -l * 2);
  glVertex3f(-w, l, -l * 2);

  glVertex3f(-l, -w, -l * 2);
  glVertex3f(-l, w, -l * 2);
  glVertex3f(l, w, -l * 2);
  glVertex3f(l, -w, -l * 2);
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
  InitTransforms();

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // Uses GLUT_DOUBLE to enable double buffering
  glutInitWindowSize(w_width, w_height);

  glutCreateWindow("Plane Episode 2");

  glutDisplayFunc(DisplayFunc);
  glutKeyboardFunc(KeyboardFunc);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MotionFunc);
  glutReshapeFunc(ReshapeFunc);

  InitGL();

  glutMainLoop();

  return 0;
}
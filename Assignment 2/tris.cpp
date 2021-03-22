// CMPS 415/515, University of Louisiana at Lafayette
//
// tris.cpp : see the class handout on OpenGL for
//            documentation of this example program
//
// NOTE: No permission is given for distribution beyond
//       the 415/515 class, of this file or any derivative works.
//

#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>

#define W_WIDTH  600
#define W_HEIGHT 600

void setlights(void)
{
  GLfloat position[] = { 3.0, 3.0, 3.0, 1.0 },
             white[] = { 1.0, 1.0, 1.0, 1.0 };

  glLightfv(GL_LIGHT0, GL_POSITION, position);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  white);
}

void drawobjects(void)
{
  float color_diffuse[]  = { 0.8, 0.5, 0.5, 1.0 },
                   p0[3] = {0.75, -0.75, 0.5},
                   p1[3] = {0.75,  0.75, 0.5},
                   p2[3] = {0.0,   0.0,  0.5};
  
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color_diffuse);

  glEnable(GL_LIGHTING);

  glBegin(GL_TRIANGLES);
    glNormal3f(0.0, 0.0, 1.0);
    glVertex3fv(p0);
	  glVertex3fv(p1);
	  glVertex3fv(p2);
  glEnd();

  glDisable(GL_LIGHTING);

  glBegin(GL_TRIANGLES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(-0.75, -0.75, 0.0);

	  glColor3f(0.0, 1.0, 0.0);
	  glVertex3f(0.0, 0.0, 0.0);
	
	  glColor3f(0.0, 0.0, 1.0);
	  glVertex3f(-0.75, 0.75, 0.0);
  glEnd();
}

void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-1.0, 1.0, -1.0, 1.0, -10.0, 10.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0.0, 0.0, 1.0,
	        0.0, 0.0, 0.0, 
	 	    0.0, 1.0, 0.0);

  setlights(); 
 
  drawobjects();

  glFlush();
}

void reshape(int w, int h)
{
  glViewport(0, 0, (GLsizei) w, (GLsizei) h);
}

void init(void)
{
  glClearColor(0.7, 0.7, 0.7, 0.0); 
  glEnable(GL_DEPTH_TEST); 
  glEnable(GL_LIGHT0);
  glShadeModel(GL_SMOOTH);
}

int main(int argc, char **argv)
{ 
  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(W_WIDTH, W_HEIGHT);
  
  glutCreateWindow("CMPS 415/515");

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  
  init();

  glutMainLoop();

  return 0;
}

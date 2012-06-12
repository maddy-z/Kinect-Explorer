#include <cstdio>
#include <cstdlib>

#include <Glut\glut.h>

#include "Pos.h"
#include "Vec.h"

// 
// Global Variables
// 

int glutMainWndHandler		= -1;

int mainWndWidth				= 800;
int mainWndHeight			= 600;
int mainWndPosX				= 150;
int mainWndPosY				= 150;

struct Pos3f	bottomLN		=	{ -5.0f, -5.0f, 0.0f }, 
					bottomRN		=	{   5.0f, -5.0f, 0.0f }, 
					bottomLF		=	{ -5.0f,   5.0f, 0.0f }, 
					bottomRF		=	{   5.0f,   5.0f, 0.0f };

struct Pos3f	topLN			=	{ -5.0f, -5.0f, 2.0f }, 
					topRN			=	{   5.0f, -5.0f, 2.0f }, 
					topLF			=	{ -5.0f,   5.0f, 2.0f }, 
					topRF			=	{   5.0f,   5.0f, 2.0f };

// 
// Camera Position Information
// 

float	cameraPosX				= 15.0f;
float cameraPosY				= 15.0f;
float cameraPosZ				= 15.0f;

//
// Initialize Function
// 

int Init ( void )
{
	glShadeModel ( GL_SMOOTH );
	glEnable ( GL_DEPTH_TEST );
	glHint ( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

	glViewport ( 0, 0, mainWndWidth, mainWndHeight );

	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluPerspective ( 45.0f, (GLdouble)(mainWndWidth) / (GLdouble)(mainWndHeight), 1.0f, 40.0f );

	return true;
}

//
// Reshape Function
//

void Reshape ( int width, int height )
{
	if ( height == 0 ) {
		height = 1;
		return;
	}

	glViewport ( 0, 0, width, height );
	
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluPerspective ( 45.0f, ( GLdouble ) ( width ) / ( GLdouble ) ( height ), 1.0f, 40.0f );
}

//
// Main Loop Function
// 

void Display ( void )
{
	// 
	// Initialization
	// 

	glClearColor ( 0.0f, 0.0f, 0.0f, 0.0f );
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	Vec3f up, n ( cameraPosY, -cameraPosX, 0 ), camPtr ( cameraPosX, cameraPosY, cameraPosZ );
	Vec3f::Cross3 ( up, n, camPtr );
	
	glMatrixMode ( GL_MODELVIEW );
	glLoadIdentity ();
	// gluLookAt ( cameraPosX, cameraPosY, cameraPosZ, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f );
	gluLookAt ( cameraPosX, cameraPosY, cameraPosZ, 0.0f, 0.0f, 0.0f, up.x(), up.y(), up.z() );

	glColor3f (0.7f, 0.7f, 0.3f );

	// Bottom Face
	glPushMatrix ();
	glBegin ( GL_QUADS );
		glVertex3f ( bottomLN.x, bottomLN.y, bottomLN.z );
		glVertex3f ( bottomRN.x, bottomRN.y, bottomRN.z );
		glVertex3f ( bottomRF.x, bottomRF.y, bottomRF.z );
		glVertex3f ( bottomLF.x, bottomLF.y, bottomLF.z );
	glEnd ();
	glPopMatrix ();
 	
	// Front Face
	glPushMatrix ();
	glBegin ( GL_QUADS );
		glVertex3f ( bottomLN.x, bottomLN.y, bottomLN.z );
		glVertex3f ( bottomRN.x, bottomRN.y, bottomRN.z );
		glVertex3f ( topRN.x, topRN.y, topRN.z );
		glVertex3f ( topLN.x, topLN.y, topLN.z );
	glEnd ();
	glPopMatrix ();

	// Right Face
	glPushMatrix ();
	glBegin ( GL_QUADS );
		glVertex3f ( bottomRN.x, bottomRN.y, bottomRN.z );
		glVertex3f ( bottomRF.x, bottomRF.y, bottomRF.z );
		glVertex3f ( topRF.x, topRF.y, topRF.z );
		glVertex3f ( topRN.x, topRN.y, topRN.z );
	glEnd();
	glPopMatrix();

	// Back Face
	glPushMatrix();
	glBegin ( GL_QUADS );
		glVertex3f ( bottomRF.x, bottomRF.y, bottomRF.z );
		glVertex3f ( bottomLF.x, bottomLF.y, bottomLF.z );
		glVertex3f ( topLF.x, topLF.y, topLF.z );
		glVertex3f ( topRF.x, topRF.y, topRF.z );
	glEnd();
	glPopMatrix();

	// Left Face
	glPushMatrix();
	glBegin ( GL_QUADS );
		glVertex3f ( bottomLF.x, bottomLF.y, bottomLF.z );
		glVertex3f ( bottomLN.x, bottomLN.y, bottomLN.z );
		glVertex3f ( topLN.x, topLN.y, topLN.z );
		glVertex3f ( topLF.x, topLF.y, topLF.z );
	glEnd();
	glPopMatrix();

	// 
	// Teapot
	// 

	glColor3f ( 0.5f, 0.2f, 0.4f );
	glPushMatrix ();
	glTranslatef ( 0.0f, 0.0f, 0.5f );
	glRotatef ( 90.0f, 1.0f, 0.0f, 0.0f );
		glutSolidTeapot ( 1.0f );
	glPopMatrix();

	// 
	// Flush and Swap Buffer
	// 
	
	glutSwapBuffers ();
}

// 
// Main Function
// 

int main ( int argc, char ** argv )
{
	//
	// GLUT Initialization
	// 

	glutInit ( &argc, argv );
	glutInitDisplayMode ( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
	glutInitWindowPosition ( mainWndPosX, mainWndPosY );
	glutInitWindowSize ( mainWndWidth, mainWndHeight );

	glutMainWndHandler = glutCreateWindow ( "Simple 3D Scene" );

	glutDisplayFunc ( Display );
	glutReshapeFunc ( Reshape );

	Init();

	// 
	// Start to Glut Loop
	// 

	glutMainLoop ();

	exit ( EXIT_SUCCESS );
}
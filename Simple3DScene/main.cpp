#include <cstdio>
#include <cstdlib>

#include <Glew\glew.h>
#include <Glut\glut.h>

#include <AntTweakBar.h>

#include "Pos.h"
#include "Vec.h"

// Global Variables
int glutMainWndHandler		=	-1;

int mainWndWidth				=	800;
int mainWndHeight			=	600;

int mainWndPosX				=	150;
int mainWndPosY				=	150;

struct Pos3f	bottomLN		=	{ -5.0f, -5.0f,  0.0f }, 
					bottomRN		=	{  5.0f, -5.0f,  0.0f }, 
					bottomLF		=	{ -5.0f,  5.0f,  0.0f }, 
					bottomRF		=	{  5.0f,  5.0f,  0.0f };

struct Pos3f	topLN			=	{ -5.0f, -5.0f,  2.0f }, 
					topRN			=	{  5.0f, -5.0f,  2.0f }, 
					topLF			=	{ -5.0f,  5.0f,  2.0f }, 
					topRF			=	{  5.0f,  5.0f,  2.0f };

// Camera Position Information
float g_CameraPosX			=	1.0f;
float g_CameraPosY			=	15.0f;
float g_CameraPosZ			=	15.0f;

// Color Settings
float g_ObjMatAmbient[]	=	{ 0.7f, 0.5f, 0.8f, 1.0f };
float g_ObjMatDiffuse[]		=	{ 0.7f, 0.5f, 0.8f, 1.0f };
float g_ObjMatSpecular[]	=	{ 0.7f, 0.5f, 0.8f, 1.0f };
float g_ObjMatShininess[]	=	{ 1.0f };

float g_BoxMatAmbient[]	=	{ 0.4f, 0.2f, 0.1f, 1.0f };
float g_BoxMatDiffuse[]		=	{ 0.4f, 0.2f, 0.1f, 1.0f };
float g_BoxMatSpecular[]	=	{ 0.4f, 0.2f, 0.1f, 1.0f };
float g_BoxMatShininess[]	=	{ 1.0f };

float g_ObjZoom				=	1.4f;

const float g_Light0_Ambient[]		=	{ 0.0f, 0.0f, 0.0f, 1.0f };
const float g_Light0_Diffuse[]			=	{ 1.0f, 1.0f, 1.0f, 1.0f };
const float g_Light0_Specular[]		=	{ 1.0f, 1.0f, 1.0f, 1.0f };
const float g_Light0_Position[]		=	{ 7.0f, 3.0f, 8.0f, 1.0f };
const float g_LightModelAmbient[]	=	{ 0.2f, 0.2f, 0.2f, 1.0f };

// Initialization Function
int GlInit ( void )
{
	glShadeModel ( GL_SMOOTH );
	glHint ( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

	// glEnable ( GL_CULL_FACE );
	glEnable ( GL_DEPTH_TEST );
	glEnable ( GL_NORMALIZE );
	glEnable ( GL_LIGHTING );
	glEnable ( GL_LIGHT0 );

	// glFrontFace ( GL_CCW );
	// glCullFace ( GL_FRONT );

	glLightModelfv ( GL_LIGHT_MODEL_AMBIENT, g_LightModelAmbient );
	glLightModeli  ( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );
	glLightModeli  ( GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE );
	glLightModeli  ( GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR );

	glViewport ( 0, 0, mainWndWidth, mainWndHeight );

	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluPerspective ( 45.0f, (GLdouble)(mainWndWidth) / (GLdouble)(mainWndHeight), 1.0f, 40.0f );

	// Set Window Size for Tweak Bar
	TwWindowSize ( mainWndWidth, mainWndHeight );

	return true;
}

// Reshape Function
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

	// Send the new Window Size to AntTweakBar
    TwWindowSize ( width, height );

	// Recall Display at next frame
	glutPostRedisplay ();
}

// Main Loop Function
void Display ( void )
{
	// 
	// Initialization
	// 

	glClearDepth ( 1.0f );
	glClearColor ( 0.0f, 0.0f, 0.0f, 0.0f );
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	Vec3f up, n ( g_CameraPosY, -g_CameraPosX, 0 ), camPtr ( g_CameraPosX, g_CameraPosY, g_CameraPosZ );
	Vec3f::Cross3 ( up, n, camPtr );
	
	glMatrixMode ( GL_MODELVIEW );
	glLoadIdentity ();
	// gluLookAt ( cameraPosX, cameraPosY, cameraPosZ, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f );
	gluLookAt ( g_CameraPosX, g_CameraPosY, g_CameraPosZ, 0.0f, 0.0f, 0.0f, up.x(), up.y(), up.z() );

	glLightfv ( GL_LIGHT0, GL_AMBIENT, g_Light0_Ambient );
	glLightfv ( GL_LIGHT0, GL_DIFFUSE, g_Light0_Diffuse );
	glLightfv ( GL_LIGHT0, GL_SPECULAR, g_Light0_Specular );
	glLightfv ( GL_LIGHT0, GL_POSITION, g_Light0_Position );

	// 
	// Draw Box
	// 
	// Set Box Material
    glMaterialfv ( GL_FRONT_AND_BACK, GL_AMBIENT, g_BoxMatAmbient );
    glMaterialfv ( GL_FRONT_AND_BACK, GL_DIFFUSE, g_BoxMatDiffuse );
	glMaterialfv ( GL_FRONT_AND_BACK, GL_SPECULAR, g_BoxMatSpecular );
	glMaterialfv ( GL_FRONT_AND_BACK, GL_SHININESS, g_BoxMatShininess );

	// Bottom Face
	glPushMatrix ();
	glScalef ( 15.0f, 15.0f, 0.2f );
	glBegin ( GL_QUADS );
		glutSolidCube ( 1.0f );
		/*
		glVertex3f ( bottomLN.x, bottomLN.y, bottomLN.z );
		glVertex3f ( bottomLF.x, bottomLF.y, bottomLF.z );
		glVertex3f ( bottomRF.x, bottomRF.y, bottomRF.z );
		glVertex3f ( bottomRN.x, bottomRN.y, bottomRN.z );	
		*/
	glEnd ();
	glPopMatrix ();
 	
	// Front Face
	glPushMatrix ();
	glTranslatef ( 0.0f, -7.5f, 1.0f );
	glRotatef ( 90.0f, 1.0f, 0.0f, 0.0f );
	glScalef ( 15.0f, 2.0f, 0.2f );
	glBegin ( GL_QUADS );
		glutSolidCube ( 1.0f );
		/*
		glVertex3f ( bottomLN.x, bottomLN.y, bottomLN.z );
		glVertex3f ( topLN.x, topLN.y, topLN.z );
		glVertex3f ( topRN.x, topRN.y, topRN.z );
		glVertex3f ( bottomRN.x, bottomRN.y, bottomRN.z );
		*/
	glEnd ();
	glPopMatrix ();

	// Right Face
	glPushMatrix ();
	glTranslatef ( 7.5f, 0.0f, 1.0f );
	glRotatef ( 90.0f, 0.0f, 1.0f, 0.0f );
	glScalef ( 2.0f, 15.0f, 0.2f );
	glBegin ( GL_QUADS );
		glutSolidCube ( 1.0f );
		/*
		glVertex3f ( bottomRN.x, bottomRN.y, bottomRN.z );
		glVertex3f ( topRN.x, topRN.y, topRN.z );
		glVertex3f ( topRF.x, topRF.y, topRF.z );
		glVertex3f ( bottomRF.x, bottomRF.y, bottomRF.z );
		*/
	glEnd ();
	glPopMatrix ();

	// Back Face
	glPushMatrix ();
	glTranslatef ( 0.0f, 7.5f, 1.0f );
	glRotatef ( 90.0f, 1.0f, 0.0f, 0.0f );
	glScalef ( 15.0f, 2.0f, 0.2f );
	glBegin ( GL_QUADS );
		glutSolidCube ( 1.0f );
		/*
		glVertex3f ( bottomRF.x, bottomRF.y, bottomRF.z );
		glVertex3f ( topRF.x, topRF.y, topRF.z );
		glVertex3f ( topLF.x, topLF.y, topLF.z );
		glVertex3f ( bottomLF.x, bottomLF.y, bottomLF.z );
		*/
	glEnd ();
	glPopMatrix ();

	// Left Face
	glPushMatrix ();
	glTranslatef ( -7.5f, 0.0f, 1.0f );
	glRotatef ( 90.0f, 0.0f, 1.0f, 0.0f );
	glScalef ( 2.0f, 15.0f, 0.2f );
	glBegin ( GL_QUADS );
		glutSolidCube ( 1.0f );
		/*
		glVertex3f ( bottomLF.x, bottomLF.y, bottomLF.z );
		glVertex3f ( topLF.x, topLF.y, topLF.z );
		glVertex3f ( topLN.x, topLN.y, topLN.z );
		glVertex3f ( bottomLN.x, bottomLN.y, bottomLN.z );
		*/
	glEnd ();
	glPopMatrix ();
	
	// 
	// DrawTeapot
	// 
	// Set Box Material
    glMaterialfv ( GL_FRONT, GL_AMBIENT, g_ObjMatAmbient );
    glMaterialfv ( GL_FRONT, GL_DIFFUSE, g_ObjMatDiffuse );
	glMaterialfv ( GL_FRONT, GL_SPECULAR, g_ObjMatSpecular );
	glMaterialfv ( GL_FRONT, GL_SHININESS, g_ObjMatShininess );

	glPushMatrix ();
	glTranslatef ( 0.0f, 0.0f, 1.0f );
	glRotatef ( 90.0f, 1.0f, 0.0f, 0.0f );
		glutSolidTeapot ( g_ObjZoom );
	glPopMatrix ();

	// Draw Tweak Bars
	TwDraw ();

	// Flush and Swap Buffer
	glutSwapBuffers ();

	// Recall Display at next frame
	glutPostRedisplay ();
}

// Main Function
int main ( int argc, char ** argv )
{
	//
	// GLUT Initialization
	// 
	
	glutInit ( &argc, argv );
	glutInitDisplayMode ( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
	glutInitWindowPosition ( mainWndPosX, mainWndPosY );
	glutInitWindowSize ( mainWndWidth, mainWndHeight );

	// glutMainWndHandler = glutCreateWindow ( "Simple 3D Scene" );
	glutCreateWindow ( "Simple 3D Scene" );
	glutCreateMenu ( NULL );

	glutDisplayFunc ( Display );
	glutReshapeFunc ( Reshape );
	glutIdleFunc ( NULL );

	GlInit ();
	
	// Tw Initialization
	TwInit ( TW_OPENGL, NULL );                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

	// Set GLUT event callbacks
	// - Directly redirect GLUT mouse button events to AntTweakBar
	glutMouseFunc ( ( GLUTmousebuttonfun ) ( TwEventMouseButtonGLUT ) );
	// - Directly redirect GLUT mouse motion events to AntTweakBar
    glutMotionFunc ( ( GLUTmousemotionfun ) ( TwEventMouseMotionGLUT ) );
	// - Directly redirect GLUT mouse "passive" motion events to AntTweakBar (same as MouseMotion)
	glutPassiveMotionFunc ( ( GLUTmousemotionfun ) ( TwEventMouseMotionGLUT ) );
	// - Directly redirect GLUT key events to AntTweakBar
	glutKeyboardFunc ( ( GLUTkeyboardfun ) ( TwEventKeyboardGLUT ) );
	// - Directly redirect GLUT special key events to AntTweakBar
	glutSpecialFunc ( ( GLUTspecialfun ) ( TwEventSpecialGLUT ) );
	// - Send 'glutGetModifers' function pointer to AntTweakBar;
	//   required because the GLUT key event functions do not report key modifiers states.
	TwGLUTModifiersFunc ( glutGetModifiers );

	TwBar * bar = TwNewBar ( "TweakBar" );
	TwDefine ( " GLOBAL help='This example shows a Simple 3D scene.' " ); 

	TwAddVarRW ( bar, "Obj Zoom", TW_TYPE_FLOAT, &g_ObjZoom, " min=1.00 max=7.00 step=0.20 keyIncr=z keyDecr=Z help='Scale the object (1=original size).' ");

	TwAddVarRW ( bar, "Obj Ambient", TW_TYPE_COLOR3F, &g_ObjMatAmbient, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Diffuse", TW_TYPE_COLOR3F, &g_ObjMatDiffuse, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Specular", TW_TYPE_COLOR3F, &g_ObjMatSpecular, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Shininess", TW_TYPE_FLOAT, &g_ObjMatShininess, " min=0.00 max=128.00 step=1.00 group='Object Material' " );

	TwAddVarRW ( bar, "Box Ambient", TW_TYPE_COLOR3F, &g_BoxMatAmbient, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Diffuse", TW_TYPE_COLOR3F, &g_BoxMatDiffuse, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Specular", TW_TYPE_COLOR3F, &g_BoxMatSpecular, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Shininess", TW_TYPE_FLOAT, &g_BoxMatShininess, " min=0.00 max=128.00 step=1.00 group='Box Material' " );

	// 
	// Start to Glut Loop
	// 
	glutMainLoop ();

	// Tw Terminate
	TwTerminate ();	

	return 0;
}
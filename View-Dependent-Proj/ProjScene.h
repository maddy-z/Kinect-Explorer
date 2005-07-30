#ifndef _PROJ_SCENE_H_
#define _PROJ_SCENE_H_

#include "..\Simple3DScene\Camera.h"

// 
// Start to Project 3D Scene
// 

// void	StartProjScene ( int argc, char ** argv );

// 
// OpenGL Related Initialization
// 

int		GlInit		( void );
void	SceneInit	( void );

// 
// OpenGL Loop Function
// 

void	Display		( void );
void	IdleDisplay ( );

// 
// Callback Functions
// 

void	Reshape		( int width, int height );
void	Keyboard	( unsigned char key, int x, int y );
void	MouseMotion	( int x, int y );
void	MouseFunc	( int button, int state, int x, int y );

// 
// External Variables
// 

extern float		g_ObjSize;

extern int			glutMainWndWidth/*	=	800*/;
extern int			glutMainWndHeight/*	=	600*/;
extern const int	glutMainWndPosX/*	=	150*/;
extern const int	glutMainWndPosY/*	=	150*/;

extern float		g_CameraRotQuat[];
extern float		g_SceneRotQuat[];

extern float		g_ObjMatAmbient[];
extern float		g_ObjMatDiffuse[];
extern float		g_ObjMatSpecular[];
extern float		g_ObjMatShininess[];

extern float		g_BoxMatAmbient[];
extern float		g_BoxMatDiffuse[];
extern float		g_BoxMatSpecular[];
extern float		g_BoxMatShininess[];

// Box Size Settings
extern const float g_BoxSizeX;
extern const float g_BoxSizeY;
extern const float g_BoxSizeZ;
// extern const float g_BoxFaceThickness	=	0.1f;

extern const float g_BoxMetricSizeX;
extern const float g_BoxMetricSizeY;
// const float g_BoxMetricSizeY	=	80.0f;																	// Centimeter
extern const float g_BoxMetricSizeZ;

extern float g_CameraMetricSizeZ;

extern const float scaleFactor;

// 
// Camera Reference
// 
extern Vec3f		g_CameraPos;
extern Camera		g_Camera;

#endif
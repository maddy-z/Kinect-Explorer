#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <Glut\glut.h>

#include "Vec.h"

class Camera 

{

public:

	enum CAM_TYPE 
	{ 
		LAND_CAM, 
		AIR_CAM 
	};

	Camera ( CAM_TYPE ct = LAND_CAM );			// Default: Land Camera
	Camera ( const Vec3f & camPos, const Vec3f & camView, const Vec3f & camAlong, CAM_TYPE ct = LAND_CAM );
	virtual ~Camera ();

	void SetCameraPos ( const Vec3f & camPos ) { m_Position = camPos; }
	void SetCameraArg ( const Vec3f & camPos, const Vec3f & camView, const Vec3f & camAlong );
	void SetCameraType ( CAM_TYPE ct );
	void SetMotionSpeed ( GLfloat speed );

	Vec3f GetPosition () const;
	Vec3f GetDirView () const;
	Vec3f GetDirAlong () const;
	Vec3f GetDirUp () const;
	GLfloat GetMotionSpeed () const;
	
	void Reset ();
	void Update ();

	void Pitch ( GLfloat theta );
	void Yaw ( GLfloat theta );
	void Roll ( GLfloat theta );
		
	void Walk ( GLfloat delta, bool Wall[4] );
	void Strafe ( GLfloat delta, bool Wall[4] );
	void Fly ( GLfloat delta );

	void PrintPosition ();

private:
	
	CAM_TYPE m_CameraType;

	GLfloat m_MotionSpeed;

	Vec3f m_Position;
	Vec3f m_Along;
	Vec3f m_Up;
	Vec3f m_Forward;
};

#endif
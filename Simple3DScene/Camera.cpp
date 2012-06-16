#include	<cmath>

#include	<Glew\glew.h>
#include	<Glut\glut.h>

#include	"Camera.h"

#define		PI				3.14159265

#define		DEG2RAD			( PI / 180.0f )
#define		RAD2DEG			( 180.0f / PI )

// 
// Constructor & Destructor
// 

Camera::Camera ( CAM_TYPE ct ) 
{
	SetCameraType ( ct );
	Reset ();
}

Camera::~Camera () 
{
	;
}

void Camera::SetCameraArg ( const Vec3f & camPos, const Vec3f & camView, const Vec3f & camAlong )
{
	m_Position = camPos;
	m_Forward = camView;
	m_Along = camAlong;

	Vec3f::Cross3 ( m_Up, m_Along, m_Forward );

	m_Forward.Normalize();
	m_Along.Normalize();
	m_Up.Normalize();
}

void Camera::SetMotionSpeed ( GLfloat speed ) { m_MotionSpeed = speed; }
void Camera::SetCameraType ( CAM_TYPE ct ) { m_CameraType = ct; }

Vec3f Camera::GetPosition () const { return m_Position; }
Vec3f Camera::GetDirView () const { return m_Forward; }
Vec3f Camera::GetDirAlong () const { return m_Along; }
Vec3f Camera::GetDirUp () const { return m_Up; }

GLfloat Camera::GetMotionSpeed() const { return m_MotionSpeed; }

void Camera::Reset () 
{
	m_Position = Vec3f ( 0.0f, 0.0f, 0.0f );

	m_Along = Vec3f ( 1.0f, 0.0f, 0.0f );
	m_Up = Vec3f ( 0.0f, 1.0f, 0.0f );
	m_Forward = Vec3f ( 0.0f, 0.0f, -1.0f );

	Update ();
}

// 
// Core Functions
// 

void Camera::Update () 
{
	GLfloat x = m_Along.Dot3 ( m_Position );
	GLfloat y = m_Up.Dot3 ( m_Position );
	GLfloat z = m_Forward.Dot3 ( m_Position );

	GLfloat ViewMatrix[4][4];
	
	ViewMatrix[0][0] = m_Along.x();
	ViewMatrix[0][1] = m_Up.x();
	ViewMatrix[0][2] = - ( m_Forward.x() );
	ViewMatrix[0][3] = 0.0;

	ViewMatrix[1][0] = m_Along.y();
	ViewMatrix[1][1] = m_Up.y();
	ViewMatrix[1][2] = - ( m_Forward.y() );
	ViewMatrix[1][3] = 0.0;

	ViewMatrix[2][0] = m_Along.z();
	ViewMatrix[2][1] = m_Up.z();
	ViewMatrix[2][2] = - ( m_Forward.z() );
	ViewMatrix[2][3] = 0.0;

	ViewMatrix[3][0] = -x;
	ViewMatrix[3][1] = -y;
	ViewMatrix[3][2] = z;
	ViewMatrix[3][3] = 1.0;

	// glMatrixMode ( GL_MODELVIEW );
	// glLoadMatrixf ( (GLfloat *)(&ViewMatrix) );
	
	glMultMatrixf ( (GLfloat *)(&ViewMatrix) );
}

// 
// Rotation Functions
// 

void Camera::Yaw ( GLfloat theta ) 
{
	m_Along = m_Along * cos ( theta * DEG2RAD ) + m_Forward * sin ( theta * DEG2RAD );
	m_Along.Normalize ();
	
	Vec3f tmp; 
	Vec3f::Cross3 ( tmp, m_Along, m_Up );
	m_Forward = tmp * ( -1.0f );
	m_Forward.Normalize ();
	// m_Forward = CrossProduct ( Along, Up ) * ( -1.0f );

	Update ();
}

void Camera::Pitch ( GLfloat theta ) 
{
	// 
	// Invert UP / DOWN for Air Cameras
	// 

	if ( m_CameraType == AIR_CAM ) {
		theta = -theta;
	}
	
	m_Forward = m_Forward * cos ( theta * DEG2RAD ) + m_Up * sin ( theta * DEG2RAD );
	m_Forward.Normalize ();

	Vec3f tmp;
	Vec3f::Cross3 ( tmp, m_Forward, m_Along );
	m_Up = tmp * ( -1.0f );
	m_Up.Normalize ();

	// m_Up = CrossProduct ( Forward, Along ) * ( -1.0f );
	
	Update ();
}

void Camera::Roll ( GLfloat theta ) 
{
	if ( m_CameraType == LAND_CAM ) {
		return; 
	}										// Not for Land Camera
	
	m_Up = m_Up * cos ( theta * DEG2RAD ) + m_Along * sin ( theta * DEG2RAD );
	// m_Up = m_Up * cos ( theta * DEG2RAD ) - m_Along * sin ( theta * DEG2RAD );
	m_Up.Normalize ();

	Vec3f::Cross3 ( m_Along, m_Forward, m_Up );
	m_Along.Normalize();

	// Vec3f tmp;
	// Vec3f::Cross3 ( tmp, m_Forward, m_Up );
	// m_Along = tmp;
	// m_Along = CrossProduct ( Forward, Up );
	
	Update ();
}

// 
// Motion Functions
// 

void Camera::Walk ( GLfloat delta, bool Wall[4] ) 
{
	if ( m_CameraType == LAND_CAM ) 
	{
		m_Position -= Vec3f (	m_Forward.x() * !(Wall[0] && m_Forward.x() * delta > 0.0 || Wall[1] && m_Forward.x() * delta < 0.0),
								0.0, 
								m_Forward.z() * !(Wall[2] && m_Forward.z() * delta > 0.0 || Wall[3] && m_Forward.z() * delta < 0.0)
								) * delta;
	}
	else {
		// m_Position -= m_Forward * delta;	
		m_Position += m_Forward * delta;					// Air Camera
	}
	
	Update ();
}

void Camera::Strafe ( GLfloat delta, bool Wall[4] ) 
{
	if ( m_CameraType == LAND_CAM ) 
	{
		m_Position -= Vec3f (	m_Along.x() * !(Wall[0] && m_Along.x() * delta > 0.0 || Wall[1] && m_Along.x() * delta < 0.0),
								0.0, 
								m_Along.z() * !(Wall[2] && m_Along.z() * delta > 0.0 || Wall[3] && m_Along.z() * delta < 0.0)
								) * delta;
	}
	else {
		m_Position += m_Along * delta;						// Air Camera
		Update ();
	}
}

void Camera::Fly ( GLfloat delta ) 
{
	// Don't Allow for Land Camera.
	if ( m_CameraType == LAND_CAM ) { return; }
	
	m_Position += m_Up * delta;
	Update ();
}


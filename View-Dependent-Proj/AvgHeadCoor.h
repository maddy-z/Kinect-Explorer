#ifndef _AVG_HEADCOOR_H_
#define _AVG_HEADCOOR_H_

#include <cmath>
#include <cassert>

#include <queue>

#include <opencv2\opencv.hpp>

class AvgHeadCoor

{

public:
	
	// 
	// Ctor & Dtor
	// 
	AvgHeadCoor ( int maxSize, double distance ) : m_MaxSize ( maxSize ),
		m_DistanceMaxLimit ( distance )
	{
		assert ( maxSize >= 3 );
		assert ( distance > 0 );
	}
	virtual ~AvgHeadCoor() 
	{
	}

	bool InsertNewHeadCoor ( const cv::Point3f & pt )
	{
		if ( m_HeadCoorQueue.size() < m_MaxSize ) {
			m_HeadCoorQueue.push_back ( pt );
			return true;
		}
		
		assert ( m_HeadCoorQueue.size() == m_MaxSize );

		cv::Point3f avgPt;
		GetAvgHeadCoor ( avgPt );

		// No Insertion because of invalid head coordinate
		double dist = sqrt ( (avgPt.x - pt.x) * (avgPt.x - pt.x) + (avgPt.y - pt.y) * (avgPt.y - pt.y) );
		if ( dist >= m_DistanceMaxLimit ) {
			return false;
		}

		m_HeadCoorQueue.erase ( m_HeadCoorQueue.begin() );
		m_HeadCoorQueue.push_back ( pt );

		return true;
	}

	void GetAvgHeadCoor ( cv::Point3f & pt ) const
	{
		double sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
		int qSize = m_HeadCoorQueue.size();

		if (qSize == 0) {
			return;
		}

		for ( int i = 0; i < qSize; ++i ) {
			sumX += m_HeadCoorQueue[i].x;
			sumY += m_HeadCoorQueue[i].y;
			sumZ += m_HeadCoorQueue[i].z;
		}

		pt.x = sumX / (double)(qSize);
		pt.y = sumY / (double)(qSize);
		pt.z = sumZ / (double)(qSize);

		return;
	}

private:

	std::vector<cv::Point3f> m_HeadCoorQueue;

	int m_MaxSize;
	double m_DistanceMaxLimit;

};

#endif
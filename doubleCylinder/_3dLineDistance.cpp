#include "_3dLineDistance.h"

caculateLineDistance3D::caculateLineDistance3D(Eigen::Vector3f line1_point, Eigen::Vector3f line1_vector,
	Eigen::Vector3f line2_point, Eigen::Vector3f line2_vector, Eigen::Matrix3f originalCoordinate )
{
	Eigen::Vector3f cross_vector = line1_vector.cross(line2_vector);
	cross_vector.normalize();
	line1_vector.normalize();
	line2_vector.normalize();
	m_translatedCoordinate.col(0) = line1_vector;
	m_translatedCoordinate.col(1) = line2_vector;
	m_translatedCoordinate.col(2) = cross_vector;
	m_originalCoordinate = originalCoordinate;
	m_line1_point = line1_point;
	m_line2_point = line2_point;
	m_bTranslateMatrix = false;
	m_bTranslatedPoints = false;
	m_bGetDistance = false;
}
caculateLineDistance3D::caculateLineDistance3D(Eigen::Matrix3f translatedCoordinate, Eigen::Matrix3f originalCoordinate )
{
	m_translatedCoordinate = translatedCoordinate;
	m_originalCoordinate = originalCoordinate;
	m_bTranslateMatrix = false;
	m_bTranslatedPoints = false;
	m_bGetDistance = false;
}

caculateLineDistance3D::~caculateLineDistance3D()
{
}

void caculateLineDistance3D::caculatleTranslateMatrix()
{
	m_translateMatrix = m_originalCoordinate.inverse()*m_translatedCoordinate;
	m_bTranslateMatrix = true;
}

Eigen::Vector3f caculateLineDistance3D::caculatleTranslatedCoordinateValue(Eigen::Vector3f originalCoordinateValue)
{
	if (!m_bTranslateMatrix)
		caculatleTranslateMatrix();
	return m_translateMatrix.inverse() *originalCoordinateValue;
}

void caculateLineDistance3D::caculatleTranslatedCoordinateValue()
{
	if (!m_bTranslateMatrix)
		caculatleTranslateMatrix();
	m_translatedPoint1 = m_translateMatrix.inverse()*m_line1_point;
	m_translatedPoint2 = m_translateMatrix.inverse()*m_line2_point;
	m_bTranslatedPoints = true;
}

float caculateLineDistance3D::getLineDistance()
{
	if (!m_bTranslatedPoints)
		caculatleTranslatedCoordinateValue();
	m_lineDistance = abs(m_translatedPoint1.z() - m_translatedPoint2.z());
	m_bGetDistance = true;
	return m_lineDistance;
}

std::vector<Eigen::Vector3f> caculateLineDistance3D::getTranslatedPoints()
{
	if (!m_bTranslatedPoints)
		caculatleTranslatedCoordinateValue();
	std::vector<Eigen::Vector3f> vec_TranslatedPoints;
	vec_TranslatedPoints.push_back(m_translatedPoint1);
	vec_TranslatedPoints.push_back(m_translatedPoint2);
	return vec_TranslatedPoints;
}

Eigen::Vector3f caculateLineDistance3D::backMapTranslatedPoints()
{
	if (!m_bTranslatedPoints)
		caculatleTranslatedCoordinateValue();
	return m_translateMatrix*m_translatedPoint1;
}

std::vector<Eigen::Vector3f> caculateLineDistance3D::getLinePoints()
{
	std::vector<Eigen::Vector3f> vec_LinePoints;
	vec_LinePoints.push_back(m_line1_point);
	vec_LinePoints.push_back(m_line2_point);
	return vec_LinePoints;
}

Eigen::Vector3f caculateLineDistance3D::getCrossPoints()
{
	if (!m_bGetDistance)
		getLineDistance();
	m_translatedCrossPoint(0) = m_translatedPoint2(0);
	m_translatedCrossPoint(1) = m_translatedPoint1(1);
	m_translatedCrossPoint(2) = (m_translatedPoint1(2) < m_translatedPoint2(2) ? m_translatedPoint1(2) : m_translatedPoint2(2)) + 0.5*m_lineDistance;
	m_CrossPoint= m_translateMatrix*m_translatedCrossPoint;
	return m_CrossPoint;
}



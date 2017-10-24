#pragma once
#include <Eigen/Eigen>
#include <stdlib.h>
#include <vector>

//功能：计算三维空间异面直线距离
//原理：构造两条直线所构成的直角坐标系 UVWO'（note: 只获得坐标系的三个坐标轴方向。因坐标原点对求取距离无影响，固省略），将分别来自两条直线的线上点转换到 UVWO' 下，
//      则转化后的两个点在两条直线方向向量叉乘的方向上的投影距离即为两个交错直线的距离。
//
class caculateLineDistance3D
{
public:
	//功能：通过输入两条空间异面直线的几何参数来确定两条直线构成的三维笛卡尔坐标系。
	//input: 
	//	line1_point,line1_vector为第一条直线的空间几何参数，线上点和直线的方向向量（向量模可以为任意长度）
	//	line2_point,line2_vector为第二条直线的空间几何参数，线上点和直线的方向向量（向量模可以为任意长度）
	//	m_originalCoordinate 为转化前直线所在坐标系
	caculateLineDistance3D(Eigen::Vector3f line1_point, Eigen::Vector3f line1_vector, 
		Eigen::Vector3f line2_point, Eigen::Vector3f line2_vector, Eigen::Matrix3f originalCoordinate = Eigen::Matrix3f::Identity());
	
	//功能：初始化计算两交错直线距离所需的两个直角坐标系
	//input: 
	//	translatedCoordinate 通过两条直线的方向矢量构成的三维笛卡尔坐标系，右手定则。
	//	m_originalCoordinate 转换前表示两条直线的坐标系，同上
	caculateLineDistance3D(Eigen::Matrix3f translatedCoordinate, Eigen::Matrix3f m_originalCoordinate = Eigen::Matrix3f::Identity());

	~caculateLineDistance3D();

	//功能：计算原始笛卡尔坐标系的基到两条直线所构成的坐标系的基的转换矩阵
	void caculatleTranslateMatrix();

	//功能：计算原始笛卡尔坐标系的点 转换到 两条直线所构成的坐标系的点三维坐标值
	Eigen::Vector3f caculatleTranslatedCoordinateValue(Eigen::Vector3f originalCoordinateValue);
	
	//功能：计算原始笛卡尔坐标系的点 转换到 两条直线所构成的坐标系的点三维坐标值，同上
	void caculatleTranslatedCoordinateValue();

	//功能：计算两条异面直线的距离。首先定义类，然后直接调用 getLineDistance（） 函数即可获得两直线距离；所需其他函数已在该函数中调用。
	float getLineDistance();

	//功能：获得转换后的点
	std::vector<Eigen::Vector3f> getTranslatedPoints();
	std::vector<Eigen::Vector3f> getLinePoints();

	//功能：获得空间交错直线在最短距离上的中点
	Eigen::Vector3f getCrossPoints();

	//功能：获得转换后的点在原始坐标系下的三维表示
	Eigen::Vector3f backMapTranslatedPoints();

//private:
	Eigen::Matrix3f m_originalCoordinate;//存储原始笛卡尔坐标系
	Eigen::Matrix3f m_translatedCoordinate;//存储两条直线所构成的笛卡尔坐标系
	Eigen::Matrix3f m_translateMatrix;
	bool m_bTranslateMatrix;
	Eigen::Vector3f m_line1_point;
	Eigen::Vector3f m_line2_point;
	Eigen::Vector3f m_translatedPoint1;
	Eigen::Vector3f m_translatedPoint2;
	bool m_bTranslatedPoints;
	Eigen::Vector3f m_translatedCrossPoint;
	Eigen::Vector3f m_CrossPoint;
	float m_lineDistance;
	bool m_bGetDistance;
};
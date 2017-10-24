#pragma once
#include <Eigen/Eigen>
#include <stdlib.h>
#include <vector>

//���ܣ�������ά�ռ�����ֱ�߾���
//ԭ����������ֱ�������ɵ�ֱ������ϵ UVWO'��note: ֻ�������ϵ�����������᷽��������ԭ�����ȡ������Ӱ�죬��ʡ�ԣ������ֱ���������ֱ�ߵ����ϵ�ת���� UVWO' �£�
//      ��ת�����������������ֱ�߷���������˵ķ����ϵ�ͶӰ���뼴Ϊ��������ֱ�ߵľ��롣
//
class caculateLineDistance3D
{
public:
	//���ܣ�ͨ�����������ռ�����ֱ�ߵļ��β�����ȷ������ֱ�߹��ɵ���ά�ѿ�������ϵ��
	//input: 
	//	line1_point,line1_vectorΪ��һ��ֱ�ߵĿռ伸�β��������ϵ��ֱ�ߵķ�������������ģ����Ϊ���ⳤ�ȣ�
	//	line2_point,line2_vectorΪ�ڶ���ֱ�ߵĿռ伸�β��������ϵ��ֱ�ߵķ�������������ģ����Ϊ���ⳤ�ȣ�
	//	m_originalCoordinate Ϊת��ǰֱ����������ϵ
	caculateLineDistance3D(Eigen::Vector3f line1_point, Eigen::Vector3f line1_vector, 
		Eigen::Vector3f line2_point, Eigen::Vector3f line2_vector, Eigen::Matrix3f originalCoordinate = Eigen::Matrix3f::Identity());
	
	//���ܣ���ʼ������������ֱ�߾������������ֱ������ϵ
	//input: 
	//	translatedCoordinate ͨ������ֱ�ߵķ���ʸ�����ɵ���ά�ѿ�������ϵ�����ֶ���
	//	m_originalCoordinate ת��ǰ��ʾ����ֱ�ߵ�����ϵ��ͬ��
	caculateLineDistance3D(Eigen::Matrix3f translatedCoordinate, Eigen::Matrix3f m_originalCoordinate = Eigen::Matrix3f::Identity());

	~caculateLineDistance3D();

	//���ܣ�����ԭʼ�ѿ�������ϵ�Ļ�������ֱ�������ɵ�����ϵ�Ļ���ת������
	void caculatleTranslateMatrix();

	//���ܣ�����ԭʼ�ѿ�������ϵ�ĵ� ת���� ����ֱ�������ɵ�����ϵ�ĵ���ά����ֵ
	Eigen::Vector3f caculatleTranslatedCoordinateValue(Eigen::Vector3f originalCoordinateValue);
	
	//���ܣ�����ԭʼ�ѿ�������ϵ�ĵ� ת���� ����ֱ�������ɵ�����ϵ�ĵ���ά����ֵ��ͬ��
	void caculatleTranslatedCoordinateValue();

	//���ܣ�������������ֱ�ߵľ��롣���ȶ����࣬Ȼ��ֱ�ӵ��� getLineDistance���� �������ɻ����ֱ�߾��룻���������������ڸú����е��á�
	float getLineDistance();

	//���ܣ����ת����ĵ�
	std::vector<Eigen::Vector3f> getTranslatedPoints();
	std::vector<Eigen::Vector3f> getLinePoints();

	//���ܣ���ÿռ佻��ֱ������̾����ϵ��е�
	Eigen::Vector3f getCrossPoints();

	//���ܣ����ת����ĵ���ԭʼ����ϵ�µ���ά��ʾ
	Eigen::Vector3f backMapTranslatedPoints();

//private:
	Eigen::Matrix3f m_originalCoordinate;//�洢ԭʼ�ѿ�������ϵ
	Eigen::Matrix3f m_translatedCoordinate;//�洢����ֱ�������ɵĵѿ�������ϵ
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
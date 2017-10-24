#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main(int argc, char** argv)
{
	//����һ���洢���Ƶ�PointCloud���ʵ����ʹ��PointXYZ�ṹʵ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//���õ��ƵĴ�С
	cloud->width = 10000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	//������õ�����������
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	//���ı���ʽ�洢����
int bb=	pcl::io::savePCDFileASCII("D:\\test_01_pcdASCII.pcd", *cloud);

	//�Զ����Ʒ�ʽ�洢����
	pcl::io::savePCDFileBinary("D:\\test_01_pcdBinary.pcd", *cloud);

	//��ѹ���Ķ����Ʒ�ʽ�洢����
	pcl::io::savePCDFileBinaryCompressed("D:\\test_01_pcdBinaryCompressed.pcd", *cloud);
}
//��������ļ�һ���ǵ���IOģ���е�PCD�ļ����溯������PCD֧�����ֲ�ͬ�Ĵ洢��ʽ���ı���ʽ�������Ʒ�ʽ��
//ѹ���Ķ����Ʒ�ʽ�������ַ�ʽ�У��ı���ʽ�Ķ�ȡ�ٶ��������������Ʒ�ʽ�Ķ�ȡ�ٶ���죬������ʹ��
//savePCDFileBinary����������������ݡ�
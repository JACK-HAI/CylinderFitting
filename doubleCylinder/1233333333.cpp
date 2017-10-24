#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main(int argc, char** argv)
{
	//定义一个存储点云的PointCloud类的实例，使用PointXYZ结构实例化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//设置点云的大小
	cloud->width = 10000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	//随机设置点云里点的坐标
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	//以文本方式存储点云
int bb=	pcl::io::savePCDFileASCII("D:\\test_01_pcdASCII.pcd", *cloud);

	//以二进制方式存储点云
	pcl::io::savePCDFileBinary("D:\\test_01_pcdBinary.pcd", *cloud);

	//以压缩的二进制方式存储点云
	pcl::io::savePCDFileBinaryCompressed("D:\\test_01_pcdBinaryCompressed.pcd", *cloud);
}
//保存点云文件一般是调用IO模块中的PCD文件保存函数。在PCD支持三种不同的存储方式：文本方式、二进制方式、
//压缩的二进制方式，这三种方式中，文本方式的读取速度最慢，而二进制方式的读取速度最快，建议大家使用
//savePCDFileBinary函数来保存点云数据。
#include "registration.h"

Reg2D::Reg2D(std::string src_file_name, std::string dst_file_name)
{
	/* read data from txt file */
	std::ifstream fp;
	fp.open(src_file_name);
	if (!fp.is_open())
	{
		std::cout << "Cannot open file #" << src_file_name << "#" << std::endl;
		return;
	}
	while (!fp.eof())
	{
		char buffer[32];
		int x, y;
		fp.getline(buffer, 30);
		sscanf_s(buffer, "%d, %d", &x, &y, 30);
		//std::cout << "x:" << x << "	y:" << y << std::endl;
		Eigen::Vector2f tmp(x, y);
		_pnt_A.push_back(tmp);
	}
	/*std::cout << _pnt_A.size() << std::endl;
	for (int i = 0; i < _pnt_A.size(); i++)
	{
		std::cout << _pnt_A[i].x() << " " << _pnt_A[i].y() << std::endl;
	}*/
	fp.close();

	fp.open(dst_file_name);
	if (!fp.is_open())
	{
		std::cout << "Cannot open file #" << dst_file_name << "#" << std::endl;
		return;
	}
	while (!fp.eof())
	{
		char buffer[32];
		int x, y;
		fp.getline(buffer, 30);
		sscanf_s(buffer, "%d, %d", &x, &y, 30);
		//std::cout << "x:" << x << "	y:" << y << std::endl;
		Eigen::Vector2f tmp(x, y);
		_pnt_B.push_back(tmp);
	}
	/*std::cout << _pnt_B.size() << std::endl;
	for (int i = 0; i < _pnt_B.size(); i++)
	{
		std::cout << _pnt_B[i].x() << " " << _pnt_B[i].y() << std::endl;
	}*/
	fp.close();
}

Reg2D::~Reg2D(void)
{

}

void Reg2D::Run(void)
{
	/* solve M by solving over-determined equation Ax = b with least square method */
	Eigen::MatrixXf A(2 * _pnt_A.size(), 4);
	Eigen::VectorXf b(2 * _pnt_B.size());
	for (int i = 0; i < 2 * _pnt_A.size(); i++)
	{
		if (i % 2 == 0)
		{
			A(i, 0) = _pnt_A[i / 2].x();
			A(i, 1) = _pnt_A[i / 2].y();
			A(i, 2) = 0;
			A(i, 3) = 0;
			b(i) = _pnt_B[i / 2].x();
		}
		else
		{
			A(i, 0) = 0;
			A(i, 1) = 0;
			A(i, 2) = _pnt_A[i / 2].x();
			A(i, 3) = _pnt_A[i / 2].y();
			b(i) = _pnt_B[i / 2].y();
		}
	}
	/*std::cout << A << std::endl;
	std::cout << b << std::endl;*/
	Eigen::MatrixXf x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	//std::cout << x << std::endl;
	_M(0, 0) = x(0);
	_M(0, 1) = x(1);
	_M(1, 0) = x(2);
	_M(1, 1) = x(3);
	//std::cout << _M << std::endl;

	/* warp source image */
	cv::Mat src_img = cv::imread("A.png");
	/*cv::imshow("source image", src_img);
	cv::waitKey();*/
	cv::Mat warp_img;
	for (int row = 0; row < src_img.rows; row++)
	{
		for (int col = 0; col < src_img.cols; col++)
		{
			
		}
	}

}

void test(void)
{
	Eigen::Matrix2f a;
	a << 0, 1, 2, 3;
	std::cout << a << std::endl;
	Eigen::Vector2f b;
	b << 0, 1;	//2*1 vector
	//std::cout << b << std::endl;
	Eigen::Vector2f c;
	c = a * b;
	std::cout << c << std::endl;
}
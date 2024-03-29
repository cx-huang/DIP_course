#include "registration.h"

Registration::Registration(std::string src_file_name, std::string dst_file_name)
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
	_mkdir("output\\project1");
}

Registration::~Registration(void)
{

}

void Registration::Run(void)
{
	/* solve M by solving over-determined equation Ax = b with least square method */
	// 1.create matrix A and vector b
	Eigen::MatrixXf A(2 * _pnt_A.size(), 6);
	Eigen::VectorXf b(2 * _pnt_B.size());
	for (int i = 0; i < 2 * _pnt_A.size(); i++)
	{
		if (i % 2 == 0)
		{
			A(i, 0) = _pnt_A[i / 2].x();
			A(i, 1) = _pnt_A[i / 2].y();
			A(i, 2) = 1;
			A(i, 3) = 0;
			A(i, 4) = 0;
			A(i, 5) = 0;
			b(i) = _pnt_B[i / 2].x();
		}
		else
		{
			A(i, 0) = 0;
			A(i, 1) = 0;
			A(i, 2) = 0;
			A(i, 3) = _pnt_A[i / 2].x();
			A(i, 4) = _pnt_A[i / 2].y();
			A(i, 5) = 1;
			b(i) = _pnt_B[i / 2].y();
		}
	}
	/*std::cout << A << std::endl;
	std::cout << b << std::endl;*/
	// 2.solve Ax = b using least square method
	Eigen::MatrixXf x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	//std::cout << x << std::endl;
	_M.resize(2, 3);
	_M(0, 0) = x(0);
	_M(0, 1) = x(1);
	_M(0, 2) = x(2);
	_M(1, 0) = x(3);
	_M(1, 1) = x(4);
	_M(1, 2) = x(5);
	std::cout << "matrix M: " << std::endl  << _M << std::endl;

	/* warp source image */
	// 1.load source image and label skeleton points
	cv::Mat src_img = cv::imread("A.png");
	cv::Mat src_img_dot;
	src_img.copyTo(src_img_dot);
	for (int i = 0; i < _pnt_A.size(); i++)
	{
		cv::Point p;
		p.x = _pnt_A[i].x();
		p.y = _pnt_A[i].y();
		cv::circle(src_img_dot, p, 3, cv::Scalar(255, 0, 0), -1);
	}
	cv::imwrite("output\\project1\\src_img_dot.png", src_img_dot);
	// 2.warp image using matrix _M
	cv::Mat warp_img = cv::Mat::zeros(src_img.rows, src_img.cols, src_img.type());
	cv::Mat M;
	cv::eigen2cv(_M, M);
	cv::warpAffine(src_img, warp_img, M, warp_img.size());
	cv::imwrite("output\\project1\\warp_img.png", warp_img);
	// 3.label warped skeleton points
	cv::Mat warp_img_dot;
	warp_img.copyTo(warp_img_dot);
	Eigen::Matrix3f M3x3;
	M3x3(0, 0) = _M(0, 0);
	M3x3(0, 1) = _M(0, 1);
	M3x3(0, 2) = _M(0, 2);
	M3x3(1, 0) = _M(1, 0);
	M3x3(1, 1) = _M(1, 1);
	M3x3(1, 2) = _M(1, 2);
	M3x3(2, 0) = 0;
	M3x3(2, 1) = 0;
	M3x3(2, 2) = 1;
	for (int i = 0; i < _pnt_A.size(); i++)
	{
		Eigen::Vector3f _pnt_A_hat;
		Eigen::Vector3f _pnt_A_homo;
		_pnt_A_homo(0) = _pnt_A[i].x();
		_pnt_A_homo(1) = _pnt_A[i].y();
		_pnt_A_homo(2) = 1;
		_pnt_A_hat = M3x3 * _pnt_A_homo;
		//std::cout << _pnt_A_hat << std::endl;
		cv::Point p;
		p.x = _pnt_A_hat.x();
		p.y = _pnt_A_hat.y();
		cv::circle(warp_img_dot, p, 3, cv::Scalar(255, 0, 0), -1);
	}
	cv::imwrite("output\\project1\\warp_img_dot.png", warp_img_dot);
	// 4.load destination image and label skeleton points
	cv::Mat dst_img = cv::imread("B.png");
	cv::Mat dst_img_dot;
	dst_img.copyTo(dst_img_dot);
	for (int i = 0; i < _pnt_B.size(); i++)
	{
		cv::Point p;
		p.x = _pnt_B[i].x();
		p.y = _pnt_B[i].y();
		circle(dst_img_dot, p, 3, cv::Scalar(0, 255, 0), -1);
	}
	cv::imwrite("output\\project1\\dst_img_dot.png", dst_img_dot);
	// 5.blend warped image and destination image for analysis
	double alpha = 0.5;
	double beta = 1 - alpha;
	cv::Mat blend_img;
	cv::addWeighted(warp_img_dot, alpha, dst_img_dot, beta, 0.0, blend_img);
	cv::imwrite("output\\project1\\blend_img.png", blend_img);
}

PoseEstimation::PoseEstimation(std::string pnt_3d_file_name, std::string pnt_2d_file_name)
{
	/* read data of 3d points and 2d points from txt file */
	std::ifstream fp;
	fp.open(pnt_3d_file_name);
	if (!fp.is_open())
	{
		std::cout << "Cannot open file #" << pnt_3d_file_name << "#" << std::endl;
		return;
	}
	while (!fp.eof())
	{
		char buffer[64];
		float x, y, z;
		fp.getline(buffer, 64);
		sscanf_s(buffer, "%f, %f, %f", &x, &y, &z, 64);
		Eigen::Vector4f tmp(x, y, z, 1);
		_pnt_3d_homo.push_back(tmp);
	}
	/*std::cout << _pnt_3d_homo.size() << std::endl;
	for (int i = 0; i < _pnt_3d_homo.size(); i++)
	{
		std::cout << _pnt_3d_homo[i].x() << "	" << _pnt_3d_homo[i].y() << "	" << _pnt_3d_homo[i].z() << std::endl;
	}*/
	fp.close();

	fp.open(pnt_2d_file_name);
	if (!fp.is_open())
	{
		std::cout << "Cannot open file #" << pnt_2d_file_name << "#" << std::endl;
		return;
	}
	while (!fp.eof())
	{
		char buffer[64];
		int x, y;
		fp.getline(buffer, 64);
		sscanf_s(buffer, "%d, %d", &x, &y, 64);
		Eigen::Vector3f tmp(x, y, 1);
		_pnt_2d_homo.push_back(tmp);
	}
	/*std::cout << _pnt_2d_homo.size() << std::endl;
	for (int i = 0; i < _pnt_2d_homo.size(); i++)
	{
		std::cout << _pnt_2d_homo[i].x() << "	" << _pnt_2d_homo[i].y() << std::endl;
	}*/
	fp.close();
	/* create directory for output */
	_mkdir("output\\project2");
	/* initialize matrix K */
	_K(0, 0) = 746.07;
	_K(0, 1) = 0;
	_K(0, 2) = 493.94;
	_K(1, 0) = 0;
	_K(1, 1) = 743.92;
	_K(1, 2) = 488.76;
	_K(2, 0) = 0;
	_K(2, 1) = 0;
	_K(2, 2) = 1;
	/*std::cout << _K << std::endl;*/
}

PoseEstimation::~PoseEstimation(void)
{

}

void PoseEstimation::Run(void)
{
	/* construct matrix A */
	Eigen::MatrixXf A(48, 12);
	A.setZero();
	Eigen::VectorXf b(48);
	b.setZero();
	for (int i = 0; i < _pnt_3d_homo.size(); i++)
	{
		A.block<1, 4>(i * 3, 4) = -_pnt_3d_homo[i].transpose();
		A.block<1, 4>(i * 3, 8) = _pnt_2d_homo[i].y() * _pnt_3d_homo[i].transpose();
		A.block<1, 4>(i * 3 + 1, 0) = _pnt_3d_homo[i].transpose();
		A.block<1, 4>(i * 3 + 1, 8) = -_pnt_2d_homo[i].x() * _pnt_3d_homo[i].transpose();
		A.block<1, 4>(i * 3 + 2, 0) = -_pnt_2d_homo[i].y() * _pnt_3d_homo[i].transpose();
		A.block<1, 4>(i * 3 + 2, 4) = _pnt_2d_homo[i].x() * _pnt_3d_homo[i].transpose();
	}
	//std::cout << "matrix A:" << std::endl << A << std::endl;
	/* solve x */
	Eigen::MatrixXf AT = A.transpose();
	Eigen::EigenSolver<Eigen::MatrixXf> es(AT * A);
	Eigen::MatrixXf D = es.pseudoEigenvalueMatrix();
	Eigen::MatrixXf V = es.pseudoEigenvectors();
	/*std::cout << "The eigenvalue matrix D is:" << std::endl << D << std::endl;
	std::cout << "The eigenvector matrix V is:" << std::endl << V << std::endl;*/
	int min_eig_val_idx = 0;
	for (int i = 0; i < D.rows(); i++)
	{	
		if (D(i, i) < D(min_eig_val_idx, min_eig_val_idx))
			min_eig_val_idx = i;
	}
	//std::cout << min_eig_val_idx << std::endl;
	//std::cout << "solution:" << std::endl << V.row(min_eig_val_idx).transpose() << std::endl;
	Eigen::MatrixXf P(3, 4);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			P(i, j) = (V.row(min_eig_val_idx).transpose())(i * 3 + j);
		}
	}
	/* compute [R T] */
	Eigen::MatrixXf RT = _K.inverse() * P;
	std::cout << "R:" << std::endl << RT.block<3, 3>(0, 0) << std::endl;
	std::cout << "T:" << std::endl << RT.block<3, 1>(0, 3) << std::endl;
}
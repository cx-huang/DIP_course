#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\calib3d\calib3d_c.h>	//for cvFindExtrinsicCameraParams2()
#include <opencv2\calib3d.hpp>	//for cv::projectPoints()
#include <Eigen>
#include <opencv2/core/eigen.hpp>	//for cv::eigen2cv()
#include <direct.h>	//for _mkdir()

class Reg2D
{
public:
	Reg2D(std::string src_file_name, std::string dst_file_name);
	~Reg2D(void);
	void Run(void);
private:
	std::vector<Eigen::Vector2f> _pnt_A;
	std::vector<Eigen::Vector2f> _pnt_B;
	Eigen::MatrixXf _M;
};

class PoseEstimation
{
public:
	PoseEstimation(std::string pnt_3d_file_name, std::string pnt_2d_file_name);
	~PoseEstimation(void);
	void Run(void);
private:
	std::vector<Eigen::Vector4f> _pnt_3d_homo;
	std::vector<Eigen::Vector3f> _pnt_2d_homo;
	Eigen::Matrix3f _K;
	Eigen::Matrix3f _R;
	Eigen::Vector3f _T;
};
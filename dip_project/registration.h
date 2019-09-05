#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>
#include <Eigen>

class Reg2D
{
public:
	Reg2D(std::string src_file_name, std::string dst_file_name);
	~Reg2D(void);
	void Run(void);
	
private:
	std::vector<Eigen::Vector2f> _pnt_A;
	std::vector<Eigen::Vector2f> _pnt_B;
	Eigen::Matrix2f _M;
};
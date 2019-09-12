#include "registration.h"

#ifdef DEBUG
#pragma comment(lib, "opencv_world410d.lib")
#else
#pragma comment(lib, "opencv_world410.lib")
#endif 

int main()
{
	Registration registration("points2d_A.txt", "points2d_B.txt");
	registration.Run();
	PoseEstimation pose_estimation("points3d_A.txt", "points2d_A.txt");
	pose_estimation.Run();

	return 0;
}
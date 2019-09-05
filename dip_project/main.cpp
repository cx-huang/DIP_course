#include <iostream>
#include "registration.h"

#ifdef DEBUG
#pragma comment(lib, "opencv_world410d.lib")
#else
#pragma comment(lib, "opencv_world410.lib")
#endif 

int main()
{
	Reg2D reg2d("points2d_A.txt", "points2d_B.txt");
	reg2d.Run();
	
	return 0;
}
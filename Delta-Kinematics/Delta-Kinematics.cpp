/**********************************************************************************************
 * Rotary Delta Kinematics - Version 1.1 (2021-1-17)
 * by https://github.com/12343954
 *
 * This Library is licensed under a GPLv3 License
 *
 * made with infomation from the following documents
 * - https://www.marginallyclever.com/other/samples/fk-ik-test.html
 * - https://github.com/tinkersprojects/Delta-Kinematics-Library (Algorithm is not accurate)
 * 
 * Robot looks like this:
 * - https://user-images.githubusercontent.com/1804003/103390674-b6b95c00-4b50-11eb-9f41-821d2d8cae12.png
 **********************************************************************************************/


#include <iostream>
#include "DeltaKinematics.h"

using namespace std;


#define ROBOT_btf		 500.0		// base to floor
#define ROBOT_f			  63.0		// base radius
#define ROBOT_rf		 130.0		// shoulder length
#define ROBOT_re		 400.0		// arm length
#define ROBOT_e			  35.0		// end effector radius
#define ROBOT_s			3200.0		// steps per turn


//DeltaKinematics DK(500.0,		63.0,	130.0,		400.0, 35.0,	3200.0);
DeltaKinematics DK(ROBOT_btf, ROBOT_f, ROBOT_rf, ROBOT_re, ROBOT_e, ROBOT_s);

void out_print(int error, bool is_inverse);

int main()
{
	int error = DK.forward(5.0, 10.0, 15.0);
	out_print(error, false);

	DK.x = 0;
	DK.y = 0;
	DK.z = -300;
	error = DK.inverse();
	out_print(error, true);

	error = DK.forward(100.0, 45.0, 45.0);
	out_print(error, false);

	error = DK.inverse(30, 30, 30);
	out_print(error, true);

	error = DK.forward(100, 100, 100);
	out_print(error, false);

	error = DK.forward(130, 130, 130);
	out_print(error, false);

	error = DK.forward(200, 200, 200);
	out_print(error, false);
}

void out_print(int error, bool is_inverse) {

	cout << "Result=" << error << "\n";
	cout << "OLD: xyz=[" << DK.LastXYZ.x << ", " << DK.LastXYZ.y << ", " << DK.LastXYZ.z << "],"
		<< " abc=[" << DK.LastABC.a << ", " << DK.LastABC.b << ", " << DK.LastABC.c << "]"
		<< endl;

	if (is_inverse) {
		cout << "NEW: DK.xyz=[" << DK.x << ", " << DK.y << ", " << DK.z << "] ==> "
			<< "DK.abc=[" << DK.a << ", " << DK.b << ", " << DK.c << "]\n\n"
			<< endl;
	}
	else {
		cout << "NEW: DK.abc=[" << DK.a << ", " << DK.b << ", " << DK.c << "] ==> "
			<< "DK.xyz=[" << DK.x << ", " << DK.y << ", " << DK.z << "]\n\n"
			<< endl;
	}

}

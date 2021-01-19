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


#include "DeltaKinematics.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
 //#include "WProgram.h"
#endif

#include <math.h>
#include <iostream>

#ifdef DEBUG
#include <iomanip>
#include <chrono> 
using namespace std;
using namespace std::chrono;
#endif // DEBUG


/// <summary>
/// constructor
/// </summary>
/// <param name="_btf">base to floor</param>
/// <param name="_f">base radius</param>
/// <param name="_rf">shoulder length</param>
/// <param name="_re">arm length</param>
/// <param name="_e">end effector radius</param>
/// <param name="_s">steps per turn</param>
DeltaKinematics::DeltaKinematics(double _btf, double _f, double _rf, double _re, double _e, double _s)
{
	btf = _btf;			// btf
	f = _f;				// base radius
	rf = _rf;			// shoulder length
	re = _re;			// arm length
	e = _e;				// end effector radius
	s = _s;				// steps per turn

#ifdef DEBUG
	cout << endl;
	cout << "=============Delta Settings==============" << endl;
	cout << "|" << setw(40) << "|" << endl;

	cout << "|" << setw(20) << "base to floor" << setw(8) << "(btf): " << setw(6) << btf << setw(6) << "|" << "\n"
		<< "|" << setw(20) << "base radius" << setw(8) << "(f): " << setw(6) << f << setw(6) << "|" << "\n"
		<< "|" << setw(20) << "shoulder length" << setw(8) << "(rf): " << setw(6) << rf << setw(6) << "|" << "\n"
		<< "|" << setw(20) << "arm length" << setw(8) << "(re): " << setw(6) << re << setw(6) << "|" << "\n"
		<< "|" << setw(20) << "end effector radius" << setw(8) << "(e): " << setw(6) << e << setw(6) << "|" << "\n"
		<< "|" << setw(20) << "steps per turn" << setw(8) << "(s): " << setw(6) << s << setw(6) << "|" << "\n";

	cout << "|" << setw(40) << "|" << endl;
	cout << "==============End Settings===============" << endl << endl;
#endif // DEBUG

	calc_bounds();

#ifdef DEBUG
	cout << ">>>>\t" << "Center = [" << Center.x << "," << Center.y << "," << Center.z << "]" << endl << endl;

	cout << ">>>>\t" << "Home   = [" << Home.x << "," << Home.y << "," << Home.z << "]" << endl << endl;

	cout << ">>>>\tX[V,A] = [" << setw(8) << X_Limit.Min << ", " << setw(8) << X_Limit.Max << "] mm\n"
		<< "    \tY[V,A] = [" << setw(8) << Y_Limit.Min << ", " << setw(8) << Y_Limit.Max << "] mm\n"
		<< "    \tZ[V,A] = [" << setw(8) << Z_Limit.Min << ", " << setw(8) << Z_Limit.Max << "] mm\n"
		<< endl;

	cout << ">>>>\tA[V,A] = [" << setw(8) << A_Limit.Min << ", " << setw(8) << A_Limit.Max << "] °\n"
		<< "    \tB[V,A] = [" << setw(8) << B_Limit.Min << ", " << setw(8) << B_Limit.Max << "] °\n"
		<< "    \tC[V,A] = [" << setw(8) << C_Limit.Min << ", " << setw(8) << C_Limit.Max << "] °\n"
		<< endl;

	cout << ">>>>\t" << "Resolution      =  ± " << Resolution << " mm" << endl << endl;

	cout << ">>>>\t" << "Increment Mode  =  " << (IncrementMode ? "true" : "false") << " ms" << endl << endl;
	cout << ">>>>\t" << "ETA             =  " << ETA << " ms" << endl << endl;
#endif // DEBUG

}

/// <summary>
/// forward kinematics: (thetaA, thetaB, thetaC) -> (x0, y0, z0)
/// </summary>
/// <returns></returns>
int DeltaKinematics::forward()
{
	return forward(a, b, c);
}

/// <summary>
/// forward kinematics: (thetaA, thetaB, thetaC) -> (x0, y0, z0)
/// </summary>
/// <param name="thetaA"></param>
/// <param name="thetaB"></param>
/// <param name="thetaC"></param>
/// <returns>x,y,z</returns>
int DeltaKinematics::forward(double thetaA, double thetaB, double thetaC)
{
	if (IncrementMode)
		store_last();

	x = 0.0;
	y = 0.0;
	z = 0.0;

	a = thetaA;
	b = thetaB;
	c = thetaC;

	double t = (f - e) * tan30 / 2.0;
	double dtr = pi / 180.0;

	thetaA *= dtr;
	thetaB *= dtr;
	thetaC *= dtr;

	double y1 = -(t + rf * cos(thetaA));
	double z1 = -rf * sin(thetaA);

	double y2 = (t + rf * cos(thetaB)) * sin30;
	double x2 = y2 * tan60;
	double z2 = -rf * sin(thetaB);

	double y3 = (t + rf * cos(thetaC)) * sin30;
	double x3 = -y3 * tan60;
	double z3 = -rf * sin(thetaC);

	double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

	double w1 = y1 * y1 + z1 * z1;
	double w2 = x2 * x2 + y2 * y2 + z2 * z2;
	double w3 = x3 * x3 + y3 * y3 + z3 * z3;

	// x = (a1*z + b1)/dnm
	double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
	double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

	// y = (a2*z + b2)/dnm;
	double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
	double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

	// a*z^2 + b*z + c = 0
	double aV = a1 * a1 + a2 * a2 + dnm * dnm;
	double bV = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
	double cV = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

	// discriminant
	double dV = bV * bV - 4.0 * aV * cV;
	if (dV < 0.0)
	{
		return non_existing_povar_error; // non-existing povar. return error,x,y,z
	}

	z = -0.5 * (bV + sqrt(dV)) / aV;
	x = (a1 * z + b1) / dnm;
	y = (a2 * z + b2) / dnm;

	return no_error;
}

/// <summary>
/// inverse kinematics \
/// helper functions, calculates angle thetaA (for YZ-pane)
/// </summary>
/// <param name="Angle"></param>
/// <param name="x0"></param>
/// <param name="y0"></param>
/// <param name="z0"></param>
/// <returns></returns>
int DeltaKinematics::delta_calcAngleYZ(double* Angle, double x0, double y0, double z0)
{
	double y1 = -0.5 * 0.57735 * f;	// f/2 * tan(30 deg)
	y0 -= 0.5 * 0.57735 * e;		// shift center to edge

	// z = a + b*y
	double aV = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0);
	double bV = (y1 - y0) / z0;

	// discriminant
	double dV = -(aV + bV * y1) * (aV + bV * y1) + rf * (bV * bV * rf + rf);
	if (dV < 0)
	{
		*Angle = 0;
		return non_existing_povar_error; // non-existing povar.  return {error, theta}
	}

	double yj = (y1 - aV * bV - sqrt(dV)) / (bV * bV + 1); // choosing outer povar
	double zj = aV + bV * yj;
	//*Angle = atan2(-zj, (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);
	*Angle = atan(-zj / (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);

	return no_error;  // return error, theta
}

/// <summary>
/// inverse kinematics: (x0, y0, z0) -> (thetaA, thetaB, thetaC)
/// </summary>
/// <returns></returns>
int DeltaKinematics::inverse()
{
	return inverse(x, y, z);
}

/// <summary>
/// inverse kinematics: (x0, y0, z0) -> (thetaA, thetaB, thetaC)
/// </summary>
/// <param name="x0"></param>
/// <param name="y0"></param>
/// <param name="z0"></param>
/// <returns>thetaA, thetaB, thetaC</returns>
int DeltaKinematics::inverse(double x0, double y0, double z0)
{
	if (IncrementMode)
		store_last();

	a = 0;
	b = 0;
	c = 0;

	x = x0;
	y = y0;
	z = z0;

	int status = delta_calcAngleYZ(&a, x0, y0, z0);

	if (status == no_error) {
		status = delta_calcAngleYZ(&b, x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0);
	}

	if (status == no_error) {
		status = delta_calcAngleYZ(&c, x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0);
	}

	return status;
}

/// <summary>
/// Boundary calculation for minmax of {xyz}, {abc}, center, home, resolution
/// </summary>
void DeltaKinematics::calc_bounds() {
#ifdef DEBUG
	auto start = high_resolution_clock::now();
#endif // DEBUG


	double x_max = -e - f - re - rf;
	double y_max = x_max;
	double z_max = x_max;
	double x_min = -x_max;
	double y_min = -x_max;
	double z_min = -x_max;
	double sd = 360.0 / s;
	double _x, _y;
	int _z;

	// find extents
	for (_z = 0; _z < s; ++_z) {
		double _r = _z * sd;
		int r = forward(_r, _r, _r);

		if (r == no_error) {
			if (z_min > z) z_min = z;
			if (z_max < z) z_max = z;
		}
	}
	if (z_min < -btf) z_min = -btf;
	if (z_max < -btf) z_max = -btf;

	double z_middle = (z_max + z_min) * 0.5;

	double original_dist = (z_max - z_middle);
	double dist = original_dist * 0.5;
	double sum = 0.0;
	double r[8][4];// = Array(8);
	double mint1 = 360.0;
	double maxt1 = -360.0;
	double mint2 = 360.0;
	double maxt2 = -360.0;
	double mint3 = 360.0;
	double maxt3 = -360.0;

	int index = 0;

	do {
		sum += dist;

		r[0][0] = inverse(+sum, +sum, z_middle + sum);
		r[0][1] = a;
		r[0][2] = b;
		r[0][3] = c;

		r[1][0] = inverse(+sum, -sum, z_middle + sum);
		r[1][1] = a;
		r[1][2] = b;
		r[1][3] = c;

		r[2][0] = inverse(-sum, -sum, z_middle + sum);
		r[2][1] = a;
		r[2][2] = b;
		r[2][3] = c;

		r[3][0] = inverse(-sum, +sum, z_middle + sum);
		r[3][1] = a;
		r[3][2] = b;
		r[3][3] = c;

		r[4][0] = inverse(+sum, +sum, z_middle - sum);
		r[4][1] = a;
		r[4][2] = b;
		r[4][3] = c;

		r[5][0] = inverse(+sum, -sum, z_middle - sum);
		r[5][1] = a;
		r[5][2] = b;
		r[5][3] = c;

		r[6][0] = inverse(-sum, -sum, z_middle - sum);
		r[6][1] = a;
		r[6][2] = b;
		r[6][3] = c;

		r[7][0] = inverse(-sum, +sum, z_middle - sum);
		r[7][1] = a;
		r[7][2] = b;
		r[7][3] = c;

		if (r[0][0] != no_error || r[1][0] != no_error || r[2][0] != no_error || r[3][0] != no_error ||
			r[4][0] != no_error || r[5][0] != no_error || r[6][0] != no_error || r[7][0] != no_error) {
			sum -= dist;
			dist *= 0.5;
		}
		else {
			for (int i = 0; i < 8; ++i) {
				if (mint1 > r[i][1]) mint1 = r[i][1];
				if (maxt1 < r[i][1]) maxt1 = r[i][1];
				if (mint2 > r[i][2]) mint2 = r[i][2];
				if (maxt2 < r[i][2]) maxt2 = r[i][2];
				if (mint3 > r[i][3]) mint3 = r[i][3];
				if (maxt3 < r[i][3]) maxt3 = r[i][3];
			}
		}
	} while (original_dist > sum && dist > 0.1);

	int home = forward(0, 0, 0);

	Center.x = 0.0;
	Center.y = 0.0;
	Center.z = roundoff(z_middle, 3);

	Home.x = 0.0;
	Home.y = 0.0;
	Home.z = roundoff(z, 3);

	// store home {xyz,abc} for incremental rotation 
	// means after calculating bounds and calibration, robot return to the home position.
	if (IncrementMode)
		store_last();

	X_Limit.Min = roundoff(-sum, 3);
	X_Limit.Max = roundoff(sum, 3);

	Y_Limit.Min = roundoff(-sum, 3);
	Y_Limit.Max = roundoff(sum, 3);

	Z_Limit.Min = roundoff(z_middle - sum, 3);
	Z_Limit.Max = roundoff(z_middle + sum, 3);

	A_Limit.Min = roundoff(mint1, 2);
	A_Limit.Max = roundoff(maxt1, 2);

	B_Limit.Min = roundoff(mint2, 2);
	B_Limit.Max = roundoff(maxt2, 2);

	C_Limit.Min = roundoff(mint3, 2);
	C_Limit.Max = roundoff(maxt3, 2);

	// resolution?  
	double r1 = forward(0, 0, 0);
	double r1_x = x, r1_y = y;

	double r2 = forward(sd, 0, 0);
	double r2_x = x, r2_y = y;

	_x = (r1_x - r2_x);
	_y = (r1_y - r2_y);
	sum = sqrt(_x * _x + _y * _y);

	Resolution = roundoff(sum, 3); //mm

#ifdef DEBUG
	ETA = duration_cast<microseconds>(high_resolution_clock::now() - start).count();
#endif // DEBUG
}

/// <summary>
/// Store the last {xyz, abc} value for incremental rotation
/// before stepper movement (forward/inverse)
/// </summary>
void DeltaKinematics::store_last() {

	LastXYZ.x = x;
	LastXYZ.y = y;
	LastXYZ.z = z;

	LastABC.a = a;
	LastABC.b = b;
	LastABC.c = c;

	calc_delta_xyz_abc();
}

/// <summary>
/// Set Increment Mode
/// </summary>
/// <param name="mode"></param>
/// <returns></returns>
bool DeltaKinematics::SetIncrementMode(bool mode) {
	IncrementMode = mode;
	return IncrementMode;
}

/// <summary>
/// Calculate incremental rotation: {Δx,Δy,Δz}, {Δa,Δb,Δc}
/// </summary>
void DeltaKinematics::calc_delta_xyz_abc()
{
	DELTA_XYZ.x = x - LastXYZ.x;
	DELTA_XYZ.y = y - LastXYZ.y;
	DELTA_XYZ.z = z - LastXYZ.z;

	DELTA_ABC.a = a - LastABC.a;
	DELTA_ABC.b = b - LastABC.b;
	DELTA_ABC.c = c - LastABC.c;


}

/// <summary>
/// rounding, keep N decimal places
/// </summary>
/// <param name="number"></param>
/// <param name="bits"></param>
/// <returns></returns>
double DeltaKinematics::roundoff(double number, unsigned int bits) {
	return round(number * pow(10, bits)) / pow(10, bits);
}
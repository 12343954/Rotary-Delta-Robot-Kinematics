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


#ifndef DeltaKinematics_h
#define DeltaKinematics_h

#define DEBUG

#define sqrt3		1.7320508075688772
#define pi                 3.141592653     // PI
#define sin120               sqrt3/2.0   
#define cos120					  -0.5        
#define tan60					 sqrt3
#define sin30					   0.5
#define tan30                1.0/sqrt3

#define no_error					 0
#define non_existing_povar_error	 1

class DeltaKinematics
{
public:
	struct XYZ
	{
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
	};

	struct ABC
	{
		double a = 0.0;
		double b = 0.0;
		double c = 0.0;
	};


	struct MinMax
	{
		double Min = 0.0;
		double Max = 0.0;
	};
	/// <summary>
	/// constructor
	/// </summary>
	/// <param name="_btf">: base to floor</param>
	/// <param name="_f">: base radius</param>
	/// <param name="_rf">: shoulder length</param>
	/// <param name="_re">: arm length</param>
	/// <param name="_e">: end effector radius</param>
	/// <param name="_s">: steps per turn</param>
	DeltaKinematics(double _btf, double _f, double _rf, double _re, double _e, double _s);

	int forward();
	int forward(double thetaA, double thetaB, double thetaC);
	int inverse();
	int inverse(double x0, double y0, double z0);

	bool SetIncrementMode(bool mode);


	/// <summary>
	/// Increment Mode: true/false
	/// </summary>
	bool IncrementMode = true;

	/// <summary>
	/// Robot resolution
	/// </summary>
	double Resolution = 0.0;

	/// <summary>
	/// Where is the middle of the envelope relative to the base (0,0,0)?
	/// </summary>
	XYZ Center;

	/// <summary>
	/// Where is the tool when the arms are parallel to the floor?
	/// </summary>
	XYZ Home;

	double x = 0.0;
	double y = 0.0;
	double z = 0.0;

	XYZ LastXYZ;

	MinMax X_Limit;
	MinMax Y_Limit;
	MinMax Z_Limit;


	/// <summary>
	/// ¦Á: Alpha angle
	/// </summary>
	double a = 0.0;

	/// <summary>
	/// ¦Â: Beta angle
	/// </summary>
	double b = 0.0;

	/// <summary>
	/// ¦Ã: Gamma angle
	/// </summary>
	double c = 0.0;

	ABC LastABC;

	MinMax A_Limit;
	MinMax B_Limit;
	MinMax C_Limit;

	/// <summary>
	/// IncrementMode, {¦¤x,¦¤y,¦¤z}
	/// </summary>
	XYZ DELTA_XYZ;

	/// <summary>
	/// IncrementMode, {¦¤a,¦¤b,¦¤c}
	/// </summary>
	ABC DELTA_ABC;

	/// <summary>
	/// (btf) base to floor 
	/// </summary>
	double btf = 0.0;

	/// <summary>
	/// (rf) shoulder length
	/// </summary>
	double rf = 0.0;

	/// <summary>
	/// (re) arm length
	/// </summary>
	double re = 0.0;

	/// <summary>
	/// (e) End Effector radius
	/// </summary>
	double e = 0.0;

	/// <summary>
	/// (f) Base radius
	/// </summary>
	double f = 0.0;

	/// <summary>
	/// (s) Steps per turn
	/// </summary>
	double s = 0.0;

	/// <summary>
	/// Estimated time of arrival
	/// </summary>
	double ETA = 0.0;

private:
	/// <summary>
	/// Store the last {xyz, abc} value for incremental rotation
	/// </summary>
	void store_last();

	/// <summary>
	/// Calculate incremental rotation: {¦¤x,¦¤y,¦¤z}, {¦¤a,¦¤b,¦¤c}
	/// </summary>
	void calc_delta_xyz_abc();

	/// <summary>
	/// Boundary calculation for minmax of {xyz}, {abc}, center, home, resolution
	/// </summary>
	void calc_bounds();

	int delta_calcAngleYZ(double* Angle, double x0, double y0, double z0);

	/// <summary>
	/// rounding, keep N decimal places
	/// </summary>
	/// <param name="number"></param>
	/// <param name="bits"></param>
	/// <returns></returns>
	double roundoff(double number, unsigned int bits);

};


#endif 




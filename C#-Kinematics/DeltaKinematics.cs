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

//smith, 2024-2-23
namespace CoolooAI.Robot.Kinematics
{
    public struct XYZ
    {
        public double x = 0.0;
        public double y = 0.0;
        public double z = 0.0;

        public XYZ()
        {
        }
    }

    public struct ABC
    {
        public double a = 0.0;
        public double b = 0.0;
        public double c = 0.0;

        public ABC()
        {
        }
    }


    public struct MinMax
    {
        public double Min = 0.0;
        public double Max = 0.0;

        public MinMax()
        {
        }
    }

    public class DeltaKinematics
    {
        #region const
        const double pi = 3.141592653;              // PI
        const double sin120 = 0.8660254037844;      // 0.86602540378443864676372317075294 = sqrt3 / 2.0;
        const double cos120 = -0.5;
        const double tan60 = 1.7320508075688772;    //1.7320508075688772935274463415059  =  sqrt3
        const double sin30 = 0.5;
        const double tan30 = 0.57735026918962;      // 0.5773502691896257645091487 = 1.0 / sqrt3;

        public const int no_error = 0;
        public const int non_existing_povar_error = 1;
        #endregion

        public double btf { get; private set; }
        public double f { get; private set; }
        public double rf { get; private set; }
        public double re { get; private set; }
        public double e { get; private set; }
        public double s { get; private set; }

        #region // current {x, y, z}
        public double x { get; private set; }
        public double y { get; private set; }
        public double z { get; private set; }
        #endregion

        #region // current {α, β, γ}
        /// <summary>
        /// α: Alpha angle
        /// </summary>
        public double a { get; private set; }
        /// <summary>
        /// β: Beta angle
        /// </summary>
        public double b { get; private set; }

        /// <summary>
        /// γ: Gamma angle
        /// </summary>
        public double c { get; private set; }
        #endregion

        /// <summary>
        /// Where is the middle of the envelope relative to the base (0,0,0)?
        /// </summary>
        public XYZ Center;

        /// <summary>
        /// Where is the tool when the arms are parallel to the floor?
        /// </summary>
        public XYZ Home;

        public MinMax X_Limit;
        public MinMax Y_Limit;
        public MinMax Z_Limit;

        public MinMax A_Limit;
        public MinMax B_Limit;
        public MinMax C_Limit;

        public double Resolution { get; private set; }
        public bool IncrementMode { get; set; }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="shoulder">shoulder length</param>
        /// <param name="arm">arm length</param>
        /// <param name="base_radius">base radius</param>
        /// <param name="effector_radius">end effector radius</param>
        /// <param name="base_to_floor">base to floor</param>
        /// <param name="steps_per_turn">steps per turn</param>
        public DeltaKinematics(double shoulder, double arm, double base_radius, double effector_radius, double base_to_floor, double steps_per_turn)
        {
            rf = shoulder;              // shoulder length
            re = arm;                   // arm length
            f = base_radius;            // base radius
            e = effector_radius;        // end effector radius
            btf = base_to_floor;        // btf
            s = steps_per_turn;         // steps per turn

            calc_bounds();
        }

        /// <summary>
        /// angle(α, β, γ) => position(x, y, z) 
        /// </summary>
        /// <param name="thetaA">α: Alpha angle</param>
        /// <param name="thetaB">β: Beta angle</param>
        /// <param name="thetaC">γ: Gamma angle</param>
        /// <returns>XYZ</returns>
        public int Forward(double thetaA, double thetaB, double thetaC)
        {
            //if (IncrementMode)
            //    store_last();

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

            double y1 = -(t + rf * Math.Cos(thetaA));
            double z1 = -rf * Math.Sin(thetaA);

            double y2 = (t + rf * Math.Cos(thetaB)) * sin30;
            double x2 = y2 * tan60;
            double z2 = -rf * Math.Sin(thetaB);

            double y3 = (t + rf * Math.Cos(thetaC)) * sin30;
            double x3 = -y3 * tan60;
            double z3 = -rf * Math.Sin(thetaC);

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
                return non_existing_povar_error; // non-existing povar. return {error,x,y,z}
            }

            z = -0.5 * (bV + Math.Sqrt(dV)) / aV;
            x = (a1 * z + b1) / dnm;
            y = (a2 * z + b2) / dnm;

            return no_error;
        }

        /// <summary>
        /// position(x, y, z)  => angle(α, β, γ) 
        /// </summary>
        /// <param name="x0">x</param>
        /// <param name="y0">y</param>
        /// <param name="z0">z</param>
        /// <returns>α, β, γ</returns>
        public int Inverse(double x0, double y0, double z0)
        {
            //if (IncrementMode)
            //    store_last();

            a = 0;
            b = 0;
            c = 0;

            x = x0;
            y = y0;
            z = z0;

            int status = delta_calcAngleYZ("a", x0, y0, z0);

            if (status == no_error)
            {
                status = delta_calcAngleYZ("b", x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0);
            }

            if (status == no_error)
            {
                status = delta_calcAngleYZ("c", x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0);
            }

            return status;
        }

        private int delta_calcAngleYZ(string Angle, double x0, double y0, double z0)
        {
            double y1 = -0.5 * tan30 * f; // f/2 * tan(30 deg)
            y0 -= 0.5 * tan30 * e;        // shift center to edge

            // z = a + b*y
            double aV = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0);
            double bV = (y1 - y0) / z0;

            // discriminant
            double dV = -(aV + bV * y1) * (aV + bV * y1) + rf * (bV * bV * rf + rf);
            if (dV < 0)
            {
                //Angle = 0;

                switch (Angle)
                {
                    case "a":
                        //*Angle = atan2(-zj, (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);
                        a = 0;
                        break;
                    case "b":
                        b = 0;
                        break;
                    case "c":
                        c = 0;
                        break;
                }

                return non_existing_povar_error; // non-existing povar.  return {error, theta}
            }

            double yj = (y1 - aV * bV - Math.Sqrt(dV)) / (bV * bV + 1); // choosing outer povar
            double zj = aV + bV * yj;
            switch (Angle)
            {
                case "a":
                    //*Angle = atan2(-zj, (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);
                    a = Math.Atan(-zj / (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);
                    break;
                case "b":
                    b = Math.Atan(-zj / (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);
                    break;
                case "c":
                    c = Math.Atan(-zj / (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);
                    break;
            }

            return no_error;  // return error, theta
        }


        public void calc_bounds()
        {
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
            for (_z = 0; _z < s; ++_z)
            {
                double _r = _z * sd;
                int rr = Forward(_r, _r, _r);

                if (rr == no_error)
                {
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
            //double r[8][4];// = Array(8);
            double[,] r = new double[8, 4];

            double mint1 = 360.0;
            double maxt1 = -360.0;
            double mint2 = 360.0;
            double maxt2 = -360.0;
            double mint3 = 360.0;
            double maxt3 = -360.0;

            int index = 0;

            do
            {
                sum += dist;

                r[0, 0] = Inverse(+sum, +sum, z_middle + sum);
                r[0, 1] = a;
                r[0, 2] = b;
                r[0, 3] = c;

                r[1, 0] = Inverse(+sum, -sum, z_middle + sum);
                r[1, 1] = a;
                r[1, 2] = b;
                r[1, 3] = c;

                r[2, 0] = Inverse(-sum, -sum, z_middle + sum);
                r[2, 1] = a;
                r[2, 2] = b;
                r[2, 3] = c;

                r[3, 0] = Inverse(-sum, +sum, z_middle + sum);
                r[3, 1] = a;
                r[3, 2] = b;
                r[3, 3] = c;

                r[4, 0] = Inverse(+sum, +sum, z_middle - sum);
                r[4, 1] = a;
                r[4, 2] = b;
                r[4, 3] = c;

                r[5, 0] = Inverse(+sum, -sum, z_middle - sum);
                r[5, 1] = a;
                r[5, 2] = b;
                r[5, 3] = c;

                r[6, 0] = Inverse(-sum, -sum, z_middle - sum);
                r[6, 1] = a;
                r[6, 2] = b;
                r[6, 3] = c;

                r[7, 0] = Inverse(-sum, +sum, z_middle - sum);
                r[7, 1] = a;
                r[7, 2] = b;
                r[7, 3] = c;

                if (r[0, 0] != no_error || r[1, 0] != no_error || r[2, 0] != no_error || r[3, 0] != no_error ||
                    r[4, 0] != no_error || r[5, 0] != no_error || r[6, 0] != no_error || r[7, 0] != no_error)
                {
                    sum -= dist;
                    dist *= 0.5;
                }
                else
                {
                    for (int i = 0; i < 8; ++i)
                    {
                        if (mint1 > r[i, 1]) mint1 = r[i, 1];
                        if (maxt1 < r[i, 1]) maxt1 = r[i, 1];
                        if (mint2 > r[i, 2]) mint2 = r[i, 2];
                        if (maxt2 < r[i, 2]) maxt2 = r[i, 2];
                        if (mint3 > r[i, 3]) mint3 = r[i, 3];
                        if (maxt3 < r[i, 3]) maxt3 = r[i, 3];
                    }
                }
            } while (original_dist > sum && dist > 0.1);

            int home = Forward(0, 0, 0);

            Center.x = 0.0;
            Center.y = 0.0;
            Center.z = roundoff(z_middle, 3);

            Home.x = 0.0;
            Home.y = 0.0;
            Home.z = roundoff(z, 3);

            // store home {xyz,abc} for incremental rotation 
            // means after calculating bounds and calibration, robot return to the home position.
            //if (IncrementMode)
            //    store_last();

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
            int r1 = Forward(0, 0, 0);
            double r1_x = x, r1_y = y;

            int r2 = Forward(sd, 0, 0);
            double r2_x = x, r2_y = y;

            _x = (r1_x - r2_x);
            _y = (r1_y - r2_y);
            sum = Math.Sqrt(_x * _x + _y * _y);

            Resolution = roundoff(sum, 3); //mm


        }
        double roundoff(double number, ushort bits)
        {
            //return round(number * pow(10, bits)) / pow(10, bits);
            return Math.Round(number, bits);
        }
    }
}

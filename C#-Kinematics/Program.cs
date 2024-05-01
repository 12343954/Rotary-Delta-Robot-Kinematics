using CoolooAI.Robot.Kinematics;

var delta = new DeltaKinematics(
                        shoulder: 200,
                        arm: 530,
                        base_radius: 142,
                        effector_radius: 46,
                        base_to_floor: 700,
                        steps_per_turn: 8192);

Console.WriteLine("Hello, World!");
Console.WriteLine($"\n\t\t————————————— Delta Robot ——————————————\n" +
                    $"\t\t|{"base to floor",20}{"(btf):",8}{delta.btf,6}{" ",4}|\n" +
                    $"\t\t|{"base radius",20}{"(f):",8}{delta.f,6}{" ",4}|\n" +
                    $"\t\t|{"shoulder length",20}{"(rf):",8}{delta.rf,6}{" ",4}|\n" +
                    $"\t\t|{"arm length",20}{"(re):",8}{delta.re,6}{" ",4}|\n" +
                    $"\t\t|{"end effector radius",20}{"(e):",8}{delta.e,6}{" ",4}|\n" +
                    $"\t\t|{"steps per turn",20}{"(s):",8}{delta.s,6}{" ",4}|\n" +
                    $"\t\t————————————————————————————————————————\n" +
                    $">>>\t{"Home",12}   =   [{delta.Home.x,3}, {delta.Home.y,3}, {delta.Home.z,10}]\n" +
                    $"\t{"Center",12}   =   [{delta.Center.x,3}, {delta.Center.y,3}, {delta.Center.z,10}]\n" +
                    $"\t{"Resolution",12}   =   ± {delta.Resolution,5} mm\n\n" +
                    $">>>\t{"X[V,A]",12}   =   [{delta.X_Limit.Min,10},  {delta.X_Limit.Max,10}] mm\n" +
                    $"\t{"Y[V,A]",12}   =   [{delta.Y_Limit.Min,10},  {delta.Y_Limit.Max,10}] mm\n" +
                    $"\t{"Z[V,A]",12}   =   [{delta.Z_Limit.Min,10},  {delta.Z_Limit.Max,10}] mm\n\n" +
                    $">>>\t{"A[V,A]",12}   =   [{delta.A_Limit.Min,10},  {delta.A_Limit.Max,10}] °\n" +
                    $"\t{"B[V,A]",12}   =   [{delta.B_Limit.Min,10},  {delta.B_Limit.Max,10}] °\n" +
                    $"\t{"C[V,A]",12}   =   [{delta.C_Limit.Min,10},  {delta.C_Limit.Max,10}] °\n" +
                    $"");


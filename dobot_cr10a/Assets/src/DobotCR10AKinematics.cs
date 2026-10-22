using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Port of ml_sim FK and the report-style closed-form branch method for the CR10A.
/// DH poses use millimetres and a right-handed frame. Unity's copied robot frame
/// is mirrored across Y, so conversions are applied at the boundary.
/// </summary>
public static class DobotCR10AKinematics
{
    public sealed class IKSolution
    {
        public readonly double[] jointsRad;
        public readonly double positionErrorMm;
        public readonly double orientationErrorRad;

        public IKSolution(double[] jointsRad, double positionErrorMm, double orientationErrorRad)
        {
            this.jointsRad = jointsRad;
            this.positionErrorMm = positionErrorMm;
            this.orientationErrorRad = orientationErrorRad;
        }
    }

    private static readonly double[,] Dh =
    {
        { 0, -Math.PI / 2, 176.5, 0 },
        { 607, 0, 193, -Math.PI / 2 },
        { 568, 0, -127, 0 },
        { 0, Math.PI / 2, 125, Math.PI / 2 },
        { 0, -Math.PI / 2, 125, 0 },
        { 0, 0, 111.3, 0 }
    };

    public static Matrix4x4 ForwardTcp(float[] jointsRad, Transform tcp)
    {
        var q = new double[6];
        for (int i = 0; i < 6; i++) q[i] = jointsRad[i];
        Vector3 tcpPosition = tcp != null ? tcp.localPosition : new Vector3(0, 0, 0.1f);
        Quaternion tcpRotation = tcp != null ? tcp.localRotation : Quaternion.identity;
        double[,] dhTcp = Multiply4(ForwardFlange(q), ToDhLocalMatrix(tcpPosition, tcpRotation));
        return ToUnityPose(dhTcp);
    }

    public static List<IKSolution> SolveTcp(Vector3 targetUnityPositionMm, Quaternion targetUnityRotation,
        Vector3 tcpLocalPosition, Quaternion tcpLocalRotation)
    {
        double[,] targetTcpDh = ToDhPose(targetUnityPositionMm, targetUnityRotation);
        double[,] tcpLocalDh = ToDhLocalMatrix(tcpLocalPosition, tcpLocalRotation);
        double[,] flangeTarget = Multiply4(targetTcpDh, InverseRigid(tcpLocalDh));
        return SolveFlange(flangeTarget);
    }

    private static double[,] ForwardFlange(double[] q)
    {
        double[,] result = Identity4();
        for (int i = 0; i < 6; i++)
        {
            double a = Dh[i, 0], alpha = Dh[i, 1], d = Dh[i, 2];
            double theta = q[i] + Dh[i, 3];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha), sa = Math.Sin(alpha);
            double[,] step = Identity4();
            step[0, 0] = ct;       step[0, 1] = -st * ca; step[0, 2] = st * sa;  step[0, 3] = a * ct;
            step[1, 0] = st;       step[1, 1] = ct * ca;  step[1, 2] = -ct * sa; step[1, 3] = a * st;
            step[2, 0] = 0;         step[2, 1] = sa;       step[2, 2] = ca;       step[2, 3] = d;
            result = Multiply4(result, step);
        }
        return result;
    }

    private static List<IKSolution> SolveFlange(double[,] target)
    {
        var solutions = new List<IKSolution>();
        double[,] targetRotation = RotationOf(target);

        // Target point is frame 5: SolveTcp removes the 100 mm TCP, then d6
        // removes the 111.3 mm frame-5-to-frame-6 offset.
        const double d6 = 111.3;
        double[] p05 =
        {
            target[0,3] - d6*target[0,2],
            target[1,3] - d6*target[1,2],
            target[2,3] - d6*target[2,2]
        };

        // Seed using q4=0, for which d5 extends the planar a3 reach.
        // Seed with several projections of d5 onto the a3 direction. For a
        // general q4, p05 is off the arm plane, so these are approximate and
        // must not be rejected by the exact planar constraint.
        var seeds=new List<double[]>();
        foreach(double q4Seed in new[] { -Math.PI, -3*Math.PI/4, -Math.PI/2, -Math.PI/4,
                                          0.0, Math.PI/4, Math.PI/2, 3*Math.PI/4 })
            seeds.AddRange(SolveArmBranches(p05,568.0+125.0*Math.Cos(q4Seed),false));
        foreach(double[] seed in seeds)
        {
            foreach(double wristSign in new[] { 1.0, -1.0 })
            {
                // Arm seeds contain q1..q3 only; wrist extraction writes q4..q6.
                double[] q=new double[6];
                Array.Copy(seed,q,Math.Min(seed.Length,3));
                SetWristBranch(q,targetRotation,wristSign);

                // d5 is along z4, whose direction depends on q4. Iterate the
                // report's matrix branch equations until arm and wrist agree.
                for(int iteration=0;iteration<20;iteration++)
                {
                    double[,] t04=ForwardThrough(q,4);
                    double[] p04=new double[3];
                    for(int axis=0;axis<3;axis++)
                        p04[axis]=p05[axis]-125.0*t04[axis,2];
                    List<double[]> armSolutions=SolveArmBranches(p04,568.0,true);
                    if(armSolutions.Count==0) break;
                    double[] selected=armSolutions[0];
                    double bestDistance=ArmDistanceSquared(selected,q);
                    for(int candidate=1;candidate<armSolutions.Count;candidate++)
                    {
                        double distance=ArmDistanceSquared(armSolutions[candidate],q);
                        if(distance<bestDistance) { bestDistance=distance; selected=armSolutions[candidate]; }
                    }
                    bool converged=bestDistance<1e-14;
                    for(int joint=0;joint<3;joint++) q[joint]=selected[joint];
                    SetWristBranch(q,targetRotation,wristSign);
                    if(converged) break;
                }

                double[,] check=ForwardFlange(q);
                double dx=target[0,3]-check[0,3], dy=target[1,3]-check[1,3], dz=target[2,3]-check[2,3];
                double pError=Math.Sqrt(dx*dx+dy*dy+dz*dz);
                double rError=Norm(RotationVector(Multiply(targetRotation,Transpose(RotationOf(check)))));
                bool duplicate=false;
                foreach(IKSolution existing in solutions)
                {
                    double distance=0;
                    for(int j=0;j<6;j++) { double difference=Wrap(existing.jointsRad[j]-q[j]); distance+=difference*difference; }
                    if(Math.Sqrt(distance)<1e-5) { duplicate=true; break; }
                }
                if(!duplicate) solutions.Add(new IKSolution(q,pError,rError));
            }
        }
        solutions.Sort((left, right) => left.positionErrorMm.CompareTo(right.positionErrorMm));
        return solutions;
    }

    private static double Wrap(double value) { return Math.Atan2(Math.Sin(value), Math.Cos(value)); }

    private static void SetWristBranch(double[] q, double[,] targetRotation, double wristSign)
    {
        double[,] t03=ForwardThrough(q,3);
        double[,] r36=Multiply(Transpose(RotationOf(t03)),targetRotation);
        double beta=Math.Acos(Math.Max(-1,Math.Min(1,r36[2,2])));
        double b=wristSign*beta, sb=Math.Sin(b), a, c;
        if(Math.Abs(sb)<1e-8) { a=Math.Atan2(r36[1,0],r36[0,0]); c=0; }
        else
        {
            a=Math.Atan2(r36[1,2]/sb,r36[0,2]/sb);
            c=Math.Atan2(r36[2,1]/sb,-r36[2,0]/sb);
        }
        q[3]=Wrap(a-Math.PI/2.0);
        q[4]=Wrap(-b);
        q[5]=Wrap(c);
    }

    private static List<double[]> SolveArmBranches(double[] targetPoint, double a3Length, bool enforcePlane = true)
    {
        var result=new List<double[]>();
        const double h=193.0-127.0+125.0, a2=607.0;
        double rho=Math.Sqrt(targetPoint[0]*targetPoint[0]+targetPoint[1]*targetPoint[1]);
        if(rho<Math.Abs(h)) return result;
        double phi=Math.Atan2(targetPoint[1],targetPoint[0]);
        double offset=Math.Asin(Math.Max(-1,Math.Min(1,h/rho)));
        double[] theta1s={Wrap(phi-offset),Wrap(phi-Math.PI+offset)};
        foreach(double theta1 in theta1s)
        {
            double[] p14=TransformPointInverse(DhStep(0,theta1),targetPoint);
            double x=p14[0], y=p14[1], r=Math.Sqrt(x*x+y*y);
            if(r<1e-9) continue;
            double cosQ3=(r*r-a2*a2-a3Length*a3Length)/(2*a2*a3Length);
            if(Math.Abs(cosQ3)>1+1e-6) continue;
            double q3Magnitude=Math.Acos(Math.Max(-1,Math.Min(1,cosQ3)));
            double heading=Math.Atan2(y,x);
            foreach(double q3 in new[] { q3Magnitude,-q3Magnitude })
            {
                double theta2Dh=Wrap(heading-Math.Atan2(a3Length*Math.Sin(q3),a2+a3Length*Math.Cos(q3)));
                double q2=Wrap(theta2Dh+Math.PI/2.0);
                double[] p24=TransformPointInverse(DhStep(1,q2),p14);
                if(enforcePlane && Math.Abs(p24[2]-(-127.0+125.0))>2.0) continue;
                double q3FromMatrix=Wrap(Math.Atan2(p24[1],p24[0]));
                result.Add(new[] { theta1,q2,q3FromMatrix });
            }
        }
        return result;
    }

    private static double ArmDistanceSquared(double[] arm, double[] q)
    {
        double sum=0;
        for(int i=0;i<3;i++) { double d=Wrap(arm[i]-q[i]); sum+=d*d; }
        return sum;
    }

    private static double[,] DhStep(int index, double q)
    {
        double a=Dh[index,0], alpha=Dh[index,1], d=Dh[index,2], theta=q+Dh[index,3];
        double ct=Math.Cos(theta), st=Math.Sin(theta), ca=Math.Cos(alpha), sa=Math.Sin(alpha);
        double[,] t=Identity4();
        t[0,0]=ct; t[0,1]=-st*ca; t[0,2]=st*sa; t[0,3]=a*ct;
        t[1,0]=st; t[1,1]=ct*ca; t[1,2]=-ct*sa; t[1,3]=a*st;
        t[2,0]=0; t[2,1]=sa; t[2,2]=ca; t[2,3]=d;
        return t;
    }

    private static double[,] ForwardThrough(double[] q, int jointCount)
    {
        double[,] result=Identity4();
        for(int i=0;i<jointCount;i++) result=Multiply4(result,DhStep(i,q[i]));
        return result;
    }

    private static double[] TransformPointInverse(double[,] t, double[] p)
    {
        double[] d={p[0]-t[0,3],p[1]-t[1,3],p[2]-t[2,3]};
        return new[]
        {
            t[0,0]*d[0]+t[1,0]*d[1]+t[2,0]*d[2],
            t[0,1]*d[0]+t[1,1]*d[1]+t[2,1]*d[2],
            t[0,2]*d[0]+t[1,2]*d[1]+t[2,2]*d[2]
        };
    }

    private static double[] RotationVector(double[,] r)
    {
        double cosine = Math.Max(-1, Math.Min(1, (r[0, 0] + r[1, 1] + r[2, 2] - 1) / 2));
        double angle = Math.Acos(cosine);
        double[] skew = { r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1] };
        if (angle < 1e-8) return new[] { skew[0] * 0.5, skew[1] * 0.5, skew[2] * 0.5 };
        if (Math.Abs(Math.PI - angle) < 1e-5)
        {
            double x = Math.Sqrt(Math.Max(0, (r[0, 0] + 1) * 0.5));
            double y = Math.Sqrt(Math.Max(0, (r[1, 1] + 1) * 0.5));
            double z = Math.Sqrt(Math.Max(0, (r[2, 2] + 1) * 0.5));
            if (r[0, 1] < 0) y = -y;
            if (r[0, 2] < 0) z = -z;
            double n = Math.Sqrt(x * x + y * y + z * z);
            if (n < 1e-12) return new[] { angle, 0.0, 0.0 };
            return new[] { angle * x / n, angle * y / n, angle * z / n };
        }
        double scale = angle / (2 * Math.Sin(angle));
        return new[] { scale * skew[0], scale * skew[1], scale * skew[2] };
    }

    private static Matrix4x4 ToUnityPose(double[,] dh)
    {
        var unity = Matrix4x4.identity;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) unity[r, c] = (float)(Sign(r) * dh[r, c] * Sign(c));
        unity[0, 3] = (float)dh[0, 3]; unity[1, 3] = (float)-dh[1, 3]; unity[2, 3] = (float)dh[2, 3];
        return unity;
    }

    private static double[,] ToDhPose(Vector3 unityPositionMm, Quaternion unityRotation)
    {
        Matrix4x4 unity = Matrix4x4.Rotate(unityRotation);
        double[,] dh = Identity4();
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) dh[r, c] = Sign(r) * unity[r, c] * Sign(c);
        dh[0, 3] = unityPositionMm.x; dh[1, 3] = -unityPositionMm.y; dh[2, 3] = unityPositionMm.z;
        return dh;
    }

    private static double[,] ToDhLocalMatrix(Vector3 unityPosition, Quaternion unityRotation)
    {
        Matrix4x4 unity = Matrix4x4.Rotate(unityRotation);
        double[,] dh = Identity4();
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) dh[r, c] = Sign(r) * unity[r, c] * Sign(c);
        dh[0, 3] = unityPosition.x * 1000f;
        dh[1, 3] = -unityPosition.y * 1000f;
        dh[2, 3] = unityPosition.z * 1000f;
        return dh;
    }

    private static double[,] InverseRigid(double[,] t)
    {
        double[,] inverse = Identity4();
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) inverse[r, c] = t[c, r];
        for (int r = 0; r < 3; r++)
            inverse[r, 3] = -(inverse[r, 0] * t[0, 3] + inverse[r, 1] * t[1, 3] + inverse[r, 2] * t[2, 3]);
        return inverse;
    }

    private static double[,] RotationOf(double[,] t)
    {
        var r = new double[3, 3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r[i, j] = t[i, j];
        return r;
    }

    private static double[,] Identity4()
    {
        var matrix = new double[4, 4];
        for (int i = 0; i < 4; i++) matrix[i, i] = 1;
        return matrix;
    }

    private static double[,] Multiply4(double[,] a, double[,] b)
    {
        var result = new double[4, 4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++) result[i, j] += a[i, k] * b[k, j];
        return result;
    }

    private static double[,] Transpose(double[,] a)
    {
        var t = new double[3, 3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[i, j] = a[j, i];
        return t;
    }

    private static double[,] Multiply(double[,] a, double[,] b)
    {
        var c = new double[3, 3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) c[i, j] += a[i, k] * b[k, j];
        return c;
    }

    private static double Dot(double[] a, double[] b)
    {
        double sum = 0; for (int i = 0; i < a.Length; i++) sum += a[i] * b[i]; return sum;
    }

    private static double Norm(double[] a) { return Math.Sqrt(Dot(a, a)); }
    private static double Sign(int index) { return index == 1 ? -1 : 1; }
}

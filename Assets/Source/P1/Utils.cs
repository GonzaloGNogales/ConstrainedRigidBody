using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic class for common operations.
/// </summary>
public class Utils
{
    /// <summary>
    /// Convert a Vector3 to VectorXD
    /// </summary>
    public static VectorXD ToVectorXD(Vector3 vin)
    {
        VectorXD vout = new DenseVectorXD(3);
        vout[0] = vin[0];
        vout[1] = vin[1];
        vout[2] = vin[2];

        return vout;
    }

    /// <summary>
    /// Convert a VectorXD to Vector3
    /// </summary>
    public static Vector3 ToVector3(VectorXD vin)
    {
        Vector3 vout = new Vector3();
        vout.Set((float) vin[0], 
                 (float) vin[1], 
                 (float) vin[2]);
        return vout;
    }

    /// <summary>
    /// Return a normalized quaternion
    /// </summary>
    public static Quaternion NormalizeQuaternion(Quaternion qin)
    {
        Quaternion qout = new Quaternion();
        float norm = Mathf.Sqrt(qin.x * qin.x + qin.y * qin.y + qin.z * qin.z + qin.w * qin.w);
        qout.Set((float)qin[0] / norm,
                 (float)qin[1] / norm,
                 (float)qin[2] / norm,
                 (float)qin[3] / norm);
        return qout;
    }

    /// <summary>
    /// Transform a quaternion into axis-angle
    /// </summary>
    public static Vector3 ToAxisAngle(Quaternion q)
    {
        Vector3 qv = new Vector3(q.x, q.y, q.z);
        qv.Normalize();

        return 2.0f * Mathf.Acos(q.w) * qv;
    }

    /// <summary>
    /// Transform an axis-angle into a quaternion
    /// </summary>
    public static Quaternion ToQuaternion(Vector3 v)
    {
        Quaternion q;

        float theta = v.magnitude;

        if (Mathf.Abs(theta) > 1e-6f)
        {
            float stheta2_divtheta = Mathf.Sin(theta / 2.0f) / theta;
            q = new Quaternion(stheta2_divtheta * v.x, stheta2_divtheta * v.y, stheta2_divtheta * v.z, Mathf.Cos(theta / 2.0f));
        }
        else
        {
            q = new Quaternion(0.5f * v.x, 0.5f * v.y, 0.5f * v.z, Mathf.Cos(theta / 2.0f));
        }
        q = Utils.NormalizeQuaternion(q);

        return q;
    }

    /// <summary>
    /// Return the skew-symmetric matrix corresponding to the cross product
    /// </summary>
    public static MatrixXD Skew(Vector3 v)
    {
        MatrixXD mout = new DenseMatrixXD(3, 3);
        mout[0, 1] = -v.z;
        mout[0, 2] = v.y;
        mout[1, 0] = v.z;
        mout[1, 2] = -v.x;
        mout[2, 0] = -v.y;
        mout[2, 1] = v.x;
        return mout;
    }

    /// <summary>
    /// Warp a matrix M as R * M * R^T
    /// </summary>
    public static MatrixXD WarpMatrix(Quaternion R, MatrixXD M)
    {
        MatrixXD mout = new DenseMatrixXD(3, 3);
        mout.SetRow(0, ToVectorXD(R * ToVector3(M.Row(0))));
        mout.SetRow(1, ToVectorXD(R * ToVector3(M.Row(1))));
        mout.SetRow(2, ToVectorXD(R * ToVector3(M.Row(2))));
        mout.SetColumn(0, ToVectorXD(R * ToVector3(mout.Column(0))));
        mout.SetColumn(1, ToVectorXD(R * ToVector3(mout.Column(1))));
        mout.SetColumn(2, ToVectorXD(R * ToVector3(mout.Column(2))));
        return mout;
    }

}

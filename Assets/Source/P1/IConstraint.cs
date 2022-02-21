using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic interface for any constraint.
/// </summary>
public interface IConstraint
{
    /// <summary>
    /// Initialize the constraint.
    /// </summary>
    void Initialize(int ind, PhysicsManager m);

    /// <summary>
    /// Get number of constraints.
    /// </summary>
    int GetNumConstraints();

    /// <summary>
    /// Write values of the constraints into the constraint vector.
    /// </summary>
    void GetConstraints(VectorXD c);

    /// <summary>
    /// Write constraint jacobian into the matrix.
    /// </summary>
    void GetConstraintJacobian(MatrixXD dcdx);

    /// <summary>
    /// Write force values into the force vector.
    /// </summary>
    void GetForce(VectorXD f);

    /// <summary>
    /// Write force jacobian values into the matrix.
    /// </summary>
    void GetForceJacobian(MatrixXD dfdx, MatrixXD dfdv);

}

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic interface for any simulation model.
/// </summary>
public interface ISimulable
{
    /// <summary>
    /// Initialize the simulable.
    /// </summary>
    void Initialize(int i, PhysicsManager m);

    /// <summary>
    /// Returns the number of model DOF.
    /// </summary>
    int GetNumDoFs();

    /// <summary>
    /// Write position values into the position vector.
    /// </summary>
    void GetPosition(VectorXD position);

    /// <summary>
    /// Set position values from the position vector.
    /// </summary>
    void SetPosition(VectorXD position);

    /// <summary>
    /// Advance an incremental position.
    /// </summary>
    void AdvanceIncrementalPosition(VectorXD delta);

    /// <summary>
    /// Write velocity values into the velocity vector.
    /// </summary>
    void GetVelocity(VectorXD velocity);

    /// <summary>
    /// Set velocity values from the velocity vector.
    /// </summary>
    void SetVelocity(VectorXD velocity);

    /// <summary>
    /// Write force values into the force vector.
    /// </summary>
    void GetForce(VectorXD force);

    /// <summary>
    /// Write force jacobian values into the matrix.
    /// </summary>
    void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv);

    /// <summary>
    /// Write mass values into the mass matrix.
    /// </summary>
    void GetMass(MatrixXD mass);

    /// <summary>
    /// Write inverse of mass values into the inverse mass matrix.
    /// </summary>
    void GetMassInverse(MatrixXD massInv);

    /// <summary>
    /// Fix the vector considering the fixed dofs.
    /// This method overwrites with
    /// 0's those fixed degrees of freedom.
    /// </summary>
    void FixVector(VectorXD v);

    /// <summary>
    /// Fix the matrix considering the fixed dofs.
    /// This method overwrites with 0's the columns and rows of the linear system at
    /// fixed dofs, and puts 1's in the diagonal.
    /// This is valid as long as the right hand side of the LS is
    /// also zero at fixed DOF. 
    /// </summary>
    void FixMatrix(MatrixXD M);

}

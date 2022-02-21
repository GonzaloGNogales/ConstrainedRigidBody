using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic point constraint between two rigid bodies.
/// </summary>
public class PointConstraint : MonoBehaviour, IConstraint
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public PointConstraint()
    {
        Manager = null;
    }

    #region EditorVariables

    public float Stiffness;

    public RigidBody bodyA;
    public RigidBody bodyB;

    #endregion

    #region OtherVariables

    int index;
    private PhysicsManager Manager;

    protected Vector3 pointA;
    protected Vector3 pointB;

    #endregion

    #region MonoBehaviour

    // Update is called once per frame
    void Update()
    {
        // Compute the average position
        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        Vector3 pos = 0.5f * (posA + posB);

        // Apply the position
        Transform xform = GetComponent<Transform>();
        xform.position = pos;
    }

    #endregion

    #region IConstraint

    public void Initialize(int ind, PhysicsManager m)
    {
        index = ind;
        Manager = m;

        // Initialize local positions. We assume that the object is connected to a Sphere mesh.
        Transform xform = GetComponent<Transform>();
        if (xform == null)
        {
            System.Console.WriteLine("[ERROR] Couldn't find any transform to the constraint");
        }
        else
        {
            System.Console.WriteLine("[TRACE] Succesfully found transform connected to the constraint");
        }

        // Initialize kinematics
        Vector3 pos = xform.position;

        // Local positions on objects
        pointA = (bodyA != null) ? bodyA.PointGlobalToLocal(pos) : pos;
        pointB = (bodyB != null) ? bodyB.PointGlobalToLocal(pos) : pos;

    }

    public int GetNumConstraints()
    {
        return 3;
    }

    public void GetConstraints(VectorXD c)
    {
        // TO BE COMPLETED
    }

    public void GetConstraintJacobian(MatrixXD dcdx)
    {
        // TO BE COMPLETED
    }

    public void GetForce(VectorXD force)
    {
        Vector3 pA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 pB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;

        Vector3 ForceA = -Stiffness * (pA - pB);
        Vector3 ForceB = -ForceA;
        
        if (bodyA != null)
        {
            Vector3 TorqueA = Vector3.Cross(pA - bodyA.m_pos, ForceA);
            force.SetSubVector(bodyA.index, 3, force.SubVector(bodyA.index, 3) + Utils.ToVectorXD(ForceA));
            force.SetSubVector(bodyA.index + 3, 3, force.SubVector(bodyA.index + 3, 3)
                                                   + Utils.ToVectorXD(TorqueA));
        }

        if (bodyB != null)
        {
            Vector3 TorqueB = Vector3.Cross(pB - bodyB.m_pos, ForceB);
            force.SetSubVector(bodyB.index, 3, force.SubVector(bodyB.index, 3) + Utils.ToVectorXD(ForceB));
            force.SetSubVector(bodyB.index + 3, 3, force.SubVector(bodyB.index + 3, 3)
                                                   + Utils.ToVectorXD(TorqueB));
        }
    }

    public void GetForce2(VectorXD force)
    {
        
    }

    private VectorXD GetC()
    {
        Vector3 pA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 pB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        return Utils.ToVectorXD(pA - pB);
    }

    private MatrixXD GetJa()
    {
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);
        MatrixXD dFdThetaa = DenseMatrixXD.CreateIdentity(3);
        MatrixXD dCdxa = new DenseMatrixXD(3, 6);
        dCdxa.SetSubMatrix(0, 3, 0, 3, I);
        dCdxa.SetSubMatrix(3, 3, 3, 3, -dFdThetaa);
        return dCdxa;
    }

    private MatrixXD GetJb()
    {
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);
        MatrixXD dFdThetab = DenseMatrixXD.CreateIdentity(3);
        MatrixXD dCdxb = new DenseMatrixXD(3, 6);
        dCdxb.SetSubMatrix(0, 0, 3, 3, -I);
        dCdxb.SetSubMatrix(3, 3, 3, 3, dFdThetab);
        return dCdxb;
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
    }

    #endregion

}

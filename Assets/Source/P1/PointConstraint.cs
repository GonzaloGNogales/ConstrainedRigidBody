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
        // Just save the C constraint vector to input parameter c
        c.SetSubVector(index, 3, GetC());
    }

    public void GetConstraintJacobian(MatrixXD dcdx)
    {
        // Get bodyA and bodyB Jacobians and set up dcdx matrix (only if they are not null)
        MatrixXD Ja = new DenseMatrixXD(3, 6); 
        MatrixXD Jb = new DenseMatrixXD(3, 6);
        if (bodyA != null)
        {       
            Ja = GetJa();
            dcdx.SetSubMatrix(index, bodyA.index, Ja);
        }

        if (bodyB != null)
        {
            Jb = GetJb();
            dcdx.SetSubMatrix(index, bodyB.index, Jb);
        }
    }

    public void GetForce(VectorXD force)
    {
        // Vector3 pA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        // Vector3 pB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        //
        // Vector3 ForceA = -Stiffness * (pA - pB);
        // Vector3 ForceB = -ForceA;
        //
        // if (bodyA != null)
        // {
        //     Vector3 TorqueA = Vector3.Cross(pA - bodyA.m_pos, ForceA);
        //     force.SetSubVector(bodyA.index, 3, force.SubVector(bodyA.index, 3) + Utils.ToVectorXD(ForceA));
        //     force.SetSubVector(bodyA.index + 3, 3, force.SubVector(bodyA.index + 3, 3)
        //                                            + Utils.ToVectorXD(TorqueA));
        // }
        //
        // if (bodyB != null)
        // {
        //     Vector3 TorqueB = Vector3.Cross(pB - bodyB.m_pos, ForceB);
        //     force.SetSubVector(bodyB.index, 3, force.SubVector(bodyB.index, 3) + Utils.ToVectorXD(ForceB));
        //     force.SetSubVector(bodyB.index + 3, 3, force.SubVector(bodyB.index + 3, 3)
        //                                            + Utils.ToVectorXD(TorqueB));
        // }
        
        if (bodyA != null)
        {
            VectorXD Fa = -Stiffness * GetJa().Transpose() * GetC();
            
            force.SetSubVector(bodyA.index, 3, force.SubVector(bodyA.index, 3) + Fa.SubVector(0, 3));
            force.SetSubVector(bodyA.index + 3, 3, force.SubVector(bodyA.index + 3, 3) + Fa.SubVector(3, 3));
        }
        if (bodyB != null)
        {
            VectorXD Fb = -Stiffness * GetJb().Transpose() * GetC();

            force.SetSubVector(bodyB.index, 3, force.SubVector(bodyB.index, 3) + Fb.SubVector(0, 3));
            force.SetSubVector(bodyB.index + 3, 3, force.SubVector(bodyB.index + 3, 3) + Fb.SubVector(3, 3));
        }
    }

    private VectorXD GetC()
    {
        Vector3 pA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 pB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        return Utils.ToVectorXD(pA - pB);
    }

    private MatrixXD GetJa()
    {
        // Directly transform pointA from local to global because we know that bodyA != null
        Vector3 pA = bodyA.PointLocalToGlobal(pointA);
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);
        MatrixXD dCdThetaA = Utils.Skew(- pA + bodyA.m_pos);
        
        MatrixXD dCdxa = new DenseMatrixXD(3, 6);
        dCdxa.SetSubMatrix(0, 0, I);
        dCdxa.SetSubMatrix(0, 3, dCdThetaA);
        return dCdxa;
    }

    private MatrixXD GetJb()
    {
        Vector3 pB = bodyB.PointLocalToGlobal(pointB);
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);
        MatrixXD dCdThetaB = Utils.Skew(pB - bodyB.m_pos);
        
        MatrixXD dCdxb = new DenseMatrixXD(3, 6);
        dCdxb.SetSubMatrix(0, 0, -I);
        dCdxb.SetSubMatrix(0, 3, dCdThetaB);
        return dCdxb;
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // Identity matrix useful declaration
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);
        
        // Compute Jacobians for each constraint only if a body is present
        if (bodyA != null)
        {
            Vector3 pA = bodyA.PointLocalToGlobal(pointA);
            
            // Pre-computation of dCtdThetaA = (pa - xa)* 
            MatrixXD dCtdThetaA = Utils.Skew(pA - bodyA.m_pos);
            
            // dFadxa
            MatrixXD dFadxa = - Stiffness * I;

            // dFadthetaa
            MatrixXD dFadthetaa = Stiffness * dCtdThetaA;

            // dTadxa
            MatrixXD dTadxa = - dFadthetaa;

            // dTadthetaa
            MatrixXD dTadthetaa = dFadthetaa * dCtdThetaA;

            // Fill dFdx (K) matrix
            // dFadxa
            dFdx.SetSubMatrix(bodyA.index, 
                bodyA.index, 
                dFdx.SubMatrix(bodyA.index, 3, bodyA.index, 3) + dFadxa);

            // dFadthetaa
            dFdx.SetSubMatrix(bodyA.index, 
                bodyA.index + 3, 
                dFdx.SubMatrix(bodyA.index, 3, bodyA.index + 3, 3) + dFadthetaa);
            
            // dTadxa
            dFdx.SetSubMatrix(bodyA.index + 3, 
                bodyA.index, 
                dFdx.SubMatrix(bodyA.index + 3, 3, bodyA.index, 3) + dTadxa);
            
            // dTadthetaa
            dFdx.SetSubMatrix(bodyA.index + 3, 
                bodyA.index + 3, 
                dFdx.SubMatrix(bodyA.index + 3, 3, bodyA.index + 3, 3) + dTadthetaa);

            // If there exists a bodyB attached to the constraint, add cross derivatives
            if (bodyB != null)
            {
                // dFadxa and cross derivatives computation
                MatrixXD dFadxb = - dFadxa;
                MatrixXD dFbdxa = - dFadxa;
                MatrixXD dFbdxb = dFadxa;
            
                // dFadthetaa and cross derivatives computation
                MatrixXD dFadthetab = - dFadthetaa;
                MatrixXD dFbdthetaa = - dFadthetaa;
                MatrixXD dFbdthetab = dFadthetaa;
            
                // dTadxa and cross derivatives computation
                MatrixXD dTadxb = - dTadxa;
                MatrixXD dTbdxa = - dTadxa;
                MatrixXD dTbdxb = dTadxa;
            
                // dTadthetaa and cross derivatives computation
                MatrixXD dTadthetab = - dTadthetaa;
                MatrixXD dTbdthetaa = - dTadthetaa;
                MatrixXD dTbdthetab = dTadthetaa;
                
                /* CROSS DERIVATIVES FOR dFadxa */
                // dFadxb
                dFdx.SetSubMatrix(bodyA.index, 
                    bodyB.index, 
                    dFdx.SubMatrix(bodyA.index, 3, bodyB.index, 3) + dFadxb);

                // dFbdxa
                dFdx.SetSubMatrix(bodyB.index, 
                    bodyA.index, 
                    dFdx.SubMatrix(bodyB.index, 3, bodyA.index, 3) + dFbdxa);

                // dFbdxb
                dFdx.SetSubMatrix(bodyB.index, 
                    bodyB.index, 
                    dFdx.SubMatrix(bodyB.index, 3, bodyB.index, 3) + dFbdxb);
                
                /* CROSS DERIVATIVES FOR dFadthetaa */
                // dFadthetab
                dFdx.SetSubMatrix(bodyA.index, 
                    bodyB.index + 3, 
                    dFdx.SubMatrix(bodyA.index, 3, bodyB.index + 3, 3) + dFadthetab);

                // dFbdthetaa
                dFdx.SetSubMatrix(bodyB.index, 
                    bodyA.index + 3, 
                    dFdx.SubMatrix(bodyB.index, 3, bodyA.index + 3, 3) + dFbdthetaa);

                // dFbdthetab
                dFdx.SetSubMatrix(bodyB.index, 
                    bodyB.index + 3, 
                    dFdx.SubMatrix(bodyB.index, 3, bodyB.index + 3, 3) + dFbdthetab);
                
                /* CROSS DERIVATIVES FOR dTadxa */
                // dTadxb
                dFdx.SetSubMatrix(bodyA.index + 3, 
                    bodyB.index, 
                    dFdx.SubMatrix(bodyA.index + 3, 3, bodyB.index, 3) + dTadxb);

                // dTbdxa
                dFdx.SetSubMatrix(bodyB.index + 3, 
                    bodyA.index, 
                    dFdx.SubMatrix(bodyB.index + 3, 3, bodyA.index, 3) + dTbdxa);

                // dTbdxb
                dFdx.SetSubMatrix(bodyB.index + 3, 
                    bodyB.index, 
                    dFdx.SubMatrix(bodyB.index + 3, 3, bodyB.index, 3) + dTbdxb);
                
                /* CROSS DERIVATIVES FOR dTadthetaa */
                // dTadthetab
                dFdx.SetSubMatrix(bodyA.index + 3, 
                    bodyB.index + 3, 
                    dFdx.SubMatrix(bodyA.index + 3, 3, bodyB.index + 3, 3) + dTadthetab);

                // dTbdthetaa
                dFdx.SetSubMatrix(bodyB.index + 3, 
                    bodyA.index + 3, 
                    dFdx.SubMatrix(bodyB.index + 3, 3, bodyA.index + 3, 3) + dTbdthetaa);

                // dTbdthetab
                dFdx.SetSubMatrix(bodyB.index + 3, 
                    bodyB.index + 3, 
                    dFdx.SubMatrix(bodyB.index + 3, 3, bodyB.index + 3, 3) + dTbdthetab);
            }
            
        }
        
        // If there is no bodyA but there is bodyB, the computations must be developed the opposite way,
        // computing only derivatives for b with respect to the b body itself
        else if (bodyB != null)
        {
            Vector3 pB = bodyB.PointLocalToGlobal(pointB);
            
            // Pre-computation of dCtdThetaA = (pb - xb)* 
            MatrixXD dCtdThetaB = Utils.Skew(pB - bodyB.m_pos);
            
            // dFbdxb
            MatrixXD dFbdxb = - Stiffness * I;

            // dFbdthetab
            MatrixXD dFbdthetab = Stiffness * dCtdThetaB;

            // dTbdxb
            MatrixXD dTbdxb = - dFbdthetab;

            // dTbdthetab
            MatrixXD dTbdthetab = dFbdthetab * dCtdThetaB;
            
            // Fill dFdx (K) matrix
            // dFbdxb
            dFdx.SetSubMatrix(bodyB.index, 
                bodyB.index, 
                dFdx.SubMatrix(bodyB.index, 3, bodyB.index, 3) + dFbdxb);

            // dFbdthetab
            dFdx.SetSubMatrix(bodyB.index, 
                bodyB.index + 3, 
                dFdx.SubMatrix(bodyB.index, 3, bodyB.index + 3, 3) + dFbdthetab);
            
            // dTbdxb
            dFdx.SetSubMatrix(bodyB.index + 3, 
                bodyB.index, 
                dFdx.SubMatrix(bodyB.index + 3, 3, bodyB.index, 3) + dTbdxb);
            
            // dTbdthetab
            dFdx.SetSubMatrix(bodyB.index + 3, 
                bodyB.index + 3, 
                dFdx.SubMatrix(bodyB.index + 3, 3, bodyB.index + 3, 3) + dTbdthetab);
        }
    }

    #endregion

}

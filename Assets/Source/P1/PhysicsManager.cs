using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		Paused = true;
		TimeStep = 0.01f;
		Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		IntegrationMethod = Integration.Symplectic;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Symplectic = 1,
        Implicit = 2,
        SymplecticConstraints = 3,
    };

	#region InEditorVariables

	public bool Paused;
	public float TimeStep;
    public Vector3 Gravity;
    public List<GameObject> SimObjects;
    public List<GameObject> Constraints;
    public Integration IntegrationMethod;

    #endregion

    #region OtherVariables

    private List<ISimulable> m_objs;
    private List<IConstraint> m_constraints;
    private int m_numDoFs;
    private int m_numConstraints;

    #endregion

    #region MonoBehaviour

    public void Start()
    {
        //Parse the simulable objects and initialize their state indices
        m_numDoFs = 0;
        m_objs = new List<ISimulable>(SimObjects.Count);

        foreach (GameObject obj in SimObjects)
        {
            ISimulable simobj = obj.GetComponent<ISimulable>();
            if (simobj != null)
            {
                m_objs.Add(simobj);

                // Initialize simulable object
                simobj.Initialize(m_numDoFs, this);

                // Retrieve pos and vel size
                m_numDoFs += simobj.GetNumDoFs();
            }
        }

        //Parse the constraints
        m_numConstraints = 0;
        m_constraints = new List<IConstraint>(Constraints.Count);

        foreach (GameObject obj in Constraints)
        {
            IConstraint constraint = obj.GetComponent<IConstraint>();
            if (constraint != null)
            {
                m_constraints.Add(constraint);

                // Initialize constraint
                constraint.Initialize(m_numConstraints, this);

                // Retrieve the number of constraints
                m_numConstraints += constraint.GetNumConstraints();
            }
        }

    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;

    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; // Not simulating

        // Select integration method
        switch (this.IntegrationMethod)
        {
            case Integration.Symplectic: this.stepSymplectic(); break;
            case Integration.Implicit: this.stepImplicit(); break;
            case Integration.SymplecticConstraints: this.stepSymplecticConstraints(); break;
            default:
                throw new System.Exception("[ERROR] Should never happen!");
        }
    }

    #endregion

    /// <summary>
    /// Performs a simulation step using Symplectic integration.
    /// </summary>
    private void stepSymplectic()
	{
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);

        foreach (ISimulable obj in m_objs)
        {
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }
        foreach (IConstraint constraint in m_constraints)
        {
            constraint.GetForce(f);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        v += TimeStep * (Minv * f);
        VectorXD x = TimeStep * v;

        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Implicit integration.
    /// </summary>
    private void stepImplicit()
    {
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        VectorXD b = new DenseVectorXD(m_numDoFs);
	    
        MatrixXD A = new DenseMatrixXD(m_numDoFs);
        MatrixXD M = new DenseMatrixXD(m_numDoFs);
        MatrixXD dFdx = new DenseMatrixXD(m_numDoFs);
        MatrixXD dFdv = new DenseMatrixXD(m_numDoFs);

        foreach (ISimulable obj in m_objs)
        {
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMass(M);
            obj.GetForceJacobian(dFdx, dFdv);
        }
        foreach (IConstraint constraint in m_constraints)
        {
            constraint.GetForce(f);
            constraint.GetForceJacobian(dFdx, dFdv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(M);
            obj.FixMatrix(dFdx);
            obj.FixMatrix(dFdv);
        }

        A = M - TimeStep * dFdv - TimeStep * TimeStep * dFdx;
        b = (M - TimeStep * dFdv) * v + TimeStep * f;
        v = A.Solve(b);
        VectorXD x = TimeStep * v;

        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Symplectic integration with constrained dynamics.
    /// The constraints are treated as implicit
    /// </summary>
    private void stepSymplecticConstraints()
    {
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        VectorXD c = new DenseVectorXD(m_numConstraints);
        MatrixXD M = new DenseMatrixXD(m_numDoFs);
        
        MatrixXD J = new DenseMatrixXD(m_numConstraints, m_numDoFs);

        MatrixXD A = new DenseMatrixXD(m_numDoFs + m_numConstraints);
        VectorXD b = new DenseVectorXD(m_numDoFs + m_numConstraints);
        VectorXD vWithLagrangianMultipliers = new DenseVectorXD(m_numDoFs + m_numConstraints);

        foreach (ISimulable obj in m_objs)
        {
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMass(M);
        }
        foreach (IConstraint constraint in m_constraints)
        {
            constraint.GetConstraints(c);
            constraint.GetConstraintJacobian(J);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(M);
        }

        // Build A matrix 18x18 =>
        // ( M Jt )
        // ( J  0 )
        MatrixXD Jt = J.Transpose();
        A.SetSubMatrix(0, 0, M);
        A.SetSubMatrix(0, m_numDoFs, Jt);
        A.SetSubMatrix(m_numDoFs, 0, J);
        // The last 6x6 matrix is all 0s
        
        // Build b vector (1x(nDoFs+nConstraints)) => ( f -c/dt )
        VectorXD minusCdeltaT = -c / TimeStep;
        b.SetSubVector(0, m_numDoFs, f);
        b.SetSubVector(m_numDoFs, m_numConstraints, minusCdeltaT);

        // vWithLagrangianMultipliers is a 1x18 vector with first 1x12 sub-vector as v
        // and second 1x6 solved sub-vector as lamda (lagrangian multiplier) 
        // Solve: Av = b => v = A.solve(b)
        // vWithLagrangianMultipliers = A.Solve(b);
        vWithLagrangianMultipliers = A.Inverse() * b;
        v = vWithLagrangianMultipliers.SubVector(0, m_numDoFs);
        VectorXD x = TimeStep * v;

        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(x);
            obj.SetVelocity(v);
        }
    }

}

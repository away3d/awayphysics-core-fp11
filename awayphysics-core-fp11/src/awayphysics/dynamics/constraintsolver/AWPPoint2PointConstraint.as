package awayphysics.dynamics.constraintsolver
{
	import awayphysics.dynamics.AWPRigidBody;

	import flash.geom.Vector3D;

	public class AWPPoint2PointConstraint extends AWPTypedConstraint
	{
		public function AWPPoint2PointConstraint(rbA : AWPRigidBody, pivotInA : Vector3D, rbB : AWPRigidBody = null, pivotInB : Vector3D = null)
		{
			m_rbA = rbA;
			m_rbB = rbB;

			if (rbB) {
				pointer = bullet.createP2PConstraintMethod2(rbA.pointer, rbB.pointer, pivotInA.x / _scaling, pivotInA.y / _scaling, pivotInA.z / _scaling, pivotInB.x / _scaling, pivotInB.y / _scaling, pivotInB.z / _scaling);
			} else {
				pointer = bullet.createP2PConstraintMethod1(rbA.pointer, pivotInA.x / _scaling, pivotInA.y / _scaling, pivotInA.z / _scaling);
			}
		}
	}
}
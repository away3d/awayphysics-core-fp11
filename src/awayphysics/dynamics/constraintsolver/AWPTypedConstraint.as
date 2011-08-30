package awayphysics.dynamics.constraintsolver {
	import awayphysics.AWPBase;
	import awayphysics.dynamics.AWPRigidBody;

	public class AWPTypedConstraint extends AWPBase {
		protected var m_rbA : AWPRigidBody;
		protected var m_rbB : AWPRigidBody;

		public function AWPTypedConstraint() {
		}

		public function get rigidBodyA() : AWPRigidBody {
			return m_rbA;
		}

		public function get rigidBodyB() : AWPRigidBody {
			return m_rbB;
		}
	}
}
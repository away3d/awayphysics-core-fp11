package awayphysics.dynamics.constraintsolver {
	import AWPC_Run.CModule;
	import AWPC_Run.createP2PConstraint1InC;
	import AWPC_Run.createP2PConstraint2InC;
	import awayphysics.dynamics.AWPRigidBody;
	import awayphysics.math.AWPVector3;

	import flash.geom.Vector3D;

	public class AWPPoint2PointConstraint extends AWPTypedConstraint {
		
		private var m_pivotInA:AWPVector3;
		private var m_pivotInB:AWPVector3;
		
		public function AWPPoint2PointConstraint(rbA : AWPRigidBody, pivotInA : Vector3D, rbB : AWPRigidBody = null, pivotInB : Vector3D = null) {
			super(0);
			m_rbA = rbA;
			m_rbB = rbB;
			
			if (rbB) {
				var vec1:AWPVector3 = new AWPVector3();
				vec1.sv3d = pivotInA;
				var vec2:AWPVector3 = new AWPVector3();
				vec2.sv3d = pivotInB;
				pointer = createP2PConstraint2InC(rbA.pointer, rbB.pointer, vec1.pointer, vec2.pointer);
				CModule.free(vec1.pointer);
				CModule.free(vec2.pointer);
			} else {
				vec1 = new AWPVector3();
				vec1.sv3d = pivotInA;
				pointer = createP2PConstraint1InC(rbA.pointer, vec1.pointer);
				CModule.free(vec1.pointer);
			}
			
			m_pivotInA = new AWPVector3(pointer + 300);
			m_pivotInB = new AWPVector3(pointer + 316);
		}
		
		public function get pivotInA():Vector3D {
			return m_pivotInA.sv3d;
		}

		public function set pivotInA(v:Vector3D):void {
			m_pivotInA.sv3d = v;
		}
		
		public function get pivotInB():Vector3D {
			return m_pivotInB.sv3d;
		}

		public function set pivotInB(v:Vector3D):void {
			m_pivotInB.sv3d = v;
		}
	}
}
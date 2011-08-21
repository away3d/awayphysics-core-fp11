package awayphysics.dynamics.constraintsolver
{
	import awayphysics.dynamics.AWPRigidBody;

	import flash.geom.Vector3D;

	public class AWPHingeConstraint extends AWPTypedConstraint
	{
		private var m_limit : AWPAngularLimit;

		public function AWPHingeConstraint(rbA : AWPRigidBody, pivotInA : Vector3D, axisInA : Vector3D, rbB : AWPRigidBody = null, pivotInB : Vector3D = null, axisInB : Vector3D = null, useReferenceFrameA : Boolean = false)
		{
			m_rbA = rbA;
			m_rbB = rbB;

			if (rbB) {
				pointer = bullet.createHingeConstraintMethod2(rbA.pointer, rbB.pointer, pivotInA.x / _scaling, pivotInA.y / _scaling, pivotInA.z / _scaling, pivotInB.x / _scaling, pivotInB.y / _scaling, pivotInB.z / _scaling, axisInA.x, axisInA.y, axisInA.z, axisInB.x, axisInB.y, axisInB.z, useReferenceFrameA ? 1 : 0);
			} else {
				pointer = bullet.createHingeConstraintMethod1(rbA.pointer, pivotInA.x / _scaling, pivotInA.y / _scaling, pivotInA.z / _scaling, axisInA.x, axisInA.y, axisInA.z, useReferenceFrameA ? 1 : 0);
			}

			m_limit = new AWPAngularLimit(pointer + 676);
		}

		public function setLimit(low : Number, high : Number, _softness : Number = 0.9, _biasFactor : Number = 0.3, _relaxationFactor : Number = 1.0) : void
		{
			m_limit.setLimit(low, high, _softness, _biasFactor, _relaxationFactor);
		}

		public function setAngularMotor(_enableMotor : Boolean, _targetVelocity : Number, _maxMotorImpulse : Number) : void
		{
			enableAngularMotor = _enableMotor;
			motorTargetVelocity = _targetVelocity;
			maxMotorImpulse = _maxMotorImpulse;
		}

		public function get motorTargetVelocity() : Number {
			return memUser._mrf(pointer + 668) * _scaling;
		}

		public function set motorTargetVelocity(v : Number) : void {
			memUser._mwf(pointer + 668, v / _scaling);
		}

		public function get maxMotorImpulse() : Number {
			return memUser._mrf(pointer + 672) * _scaling;
		}

		public function set maxMotorImpulse(v : Number) : void {
			memUser._mwf(pointer + 672, v / _scaling);
		}

		public function get angularOnly() : Boolean {
			return memUser._mru8(pointer + 724) == 1;
		}

		public function set angularOnly(v : Boolean) : void {
			memUser._mw8(pointer + 724, v ? 1 : 0);
		}

		public function get enableAngularMotor() : Boolean {
			return memUser._mru8(pointer + 725) == 1;
		}

		public function set enableAngularMotor(v : Boolean) : void {
			memUser._mw8(pointer + 725, v ? 1 : 0);
		}

		public function get useOffsetForConstraintFrame() : Boolean {
			return memUser._mru8(pointer + 727) == 1;
		}

		public function set useOffsetForConstraintFrame(v : Boolean) : void {
			memUser._mw8(pointer + 727, v ? 1 : 0);
		}
	}
}
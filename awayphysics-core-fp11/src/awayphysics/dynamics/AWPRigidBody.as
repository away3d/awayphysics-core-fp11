package awayphysics.dynamics
{
	import awayphysics.collision.dispatch.AWPCollisionObject;
	import awayphysics.collision.shapes.AWPShape;
	import awayphysics.data.AWPCollisionFlags;
	import awayphysics.math.AWPMatrix3x3;
	import awayphysics.math.AWPVector3;
	import awayphysics.plugin.IMesh3D;

	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;

	public class AWPRigidBody extends AWPCollisionObject
	{
		private var m_invInertiaTensorWorld : AWPMatrix3x3;
		private var m_linearVelocity : AWPVector3;
		private var m_angularVelocity : AWPVector3;
		private var m_linearFactor : AWPVector3;
		private var m_gravity : AWPVector3;
		private var m_gravity_acceleration : AWPVector3;
		private var m_invInertiaLocal : AWPVector3;
		private var m_totalForce : AWPVector3;
		private var m_totalTorque : AWPVector3;

		// rigidbody is static if mass is zero, otherwise is dynamic
		public function AWPRigidBody(shape : AWPShape, skin : IMesh3D = null, mass : Number = 0)
		{
			pointer = bullet.createBodyMethod(this, shape.pointer, mass);
			super(pointer, shape, skin);

			m_invInertiaTensorWorld = new AWPMatrix3x3(pointer + 256);
			m_linearVelocity = new AWPVector3(pointer + 304);
			m_angularVelocity = new AWPVector3(pointer + 320);
			m_linearFactor = new AWPVector3(pointer + 340);
			m_gravity = new AWPVector3(pointer + 356);
			m_gravity_acceleration = new AWPVector3(pointer + 372);
			m_invInertiaLocal = new AWPVector3(pointer + 388);
			m_totalForce = new AWPVector3(pointer + 404);
			m_totalTorque = new AWPVector3(pointer + 420);
		}

		override public function set position(pos : Vector3D) : void {
			super.position = pos;
			if (this.collisionFlags == AWPCollisionFlags.CF_STATIC_OBJECT) {
				updateTransform();
			}
		}

		override public function set rotation(rot : Matrix3D) : void {
			super.rotation = rot;
			if (this.collisionFlags == AWPCollisionFlags.CF_STATIC_OBJECT) {
				updateTransform();
			}
		}

		override public function setWorldTransform(pos : Vector3D, rot : Matrix3D) : void
		{
			super.setWorldTransform(pos, rot);
			if (this.collisionFlags == AWPCollisionFlags.CF_STATIC_OBJECT) {
				updateTransform();
			}
		}

		// add a center force to the rigidbody
		public function applyCentralForce(force : Vector3D) : void
		{
			var vec : Vector3D = force.clone();
			m_totalForce.v3d = vec.add(m_totalForce.v3d);
			activate();
		}

		// add torque to the rigidbody
		public function applyTorque(torque : Vector3D) : void
		{
			var vec : Vector3D = torque.clone();
			vec.scaleBy(1 / _scaling);
			m_totalTorque.v3d = vec.add(m_totalTorque.v3d);
			activate();
		}

		// add a force to the rigidbody
		public function applyForce(force : Vector3D, rel_pos : Vector3D) : void
		{
			applyCentralForce(force);
			applyTorque(rel_pos.crossProduct(force));
		}

		// add a center impulse to the rigidbody
		public function applyCentralImpulse(impulse : Vector3D) : void
		{
			var vec : Vector3D = impulse.clone();
			vec.scaleBy(1 / _scaling);
			vec.scaleBy(inverseMass);
			m_linearVelocity.v3d = vec.add(m_linearVelocity.v3d);
			activate();
		}

		// add a torque impulse to the rigidbody
		public function applyTorqueImpulse(torque : Vector3D) : void
		{
			var tor : Vector3D = torque.clone();
			tor.scaleBy(1 / _scaling);
			var vec : Vector3D = new Vector3D(m_invInertiaTensorWorld.col1.v3d.dotProduct(tor), m_invInertiaTensorWorld.col2.v3d.dotProduct(tor), m_invInertiaTensorWorld.col3.v3d.dotProduct(tor));
			m_angularVelocity.v3d = vec.add(m_angularVelocity.v3d);
			activate();
		}

		// add a impulse to the rigidbody
		public function applyImpulse(impulse : Vector3D, rel_pos : Vector3D) : void
		{
			if (inverseMass != 0) {
				applyCentralImpulse(impulse);
				applyTorqueImpulse(rel_pos.crossProduct(impulse));
			}
		}

		// clear all force and torque to zero
		public function clearForces() : void
		{
			m_totalForce.v3d = new Vector3D();
			m_totalTorque.v3d = new Vector3D();
		}

		// set the gravity of this rigidbody
		public function set gravity(acceleration : Vector3D) : void {
			if (inverseMass != 0) {
				acceleration.scaleBy(1 / inverseMass);
				m_gravity.v3d = acceleration;
				activate();
			}
			m_gravity_acceleration.v3d = acceleration;
		}

		public function get invInertiaTensorWorld() : Matrix3D {
			return m_invInertiaTensorWorld.m3d;
		}

		public function get linearVelocity() : Vector3D {
			return m_linearVelocity.sv3d;
		}

		public function set linearVelocity(v : Vector3D) : void {
			m_linearVelocity.sv3d = v;
		}

		public function get angularVelocity() : Vector3D {
			return m_angularVelocity.v3d;
		}

		public function set angularVelocity(v : Vector3D) : void {
			m_angularVelocity.v3d = v;
		}

		public function get linearFactor() : Vector3D {
			return m_linearFactor.v3d;
		}

		public function get gravity() : Vector3D {
			return m_gravity.v3d;
		}

		public function get gravityAcceleration() : Vector3D {
			return m_gravity_acceleration.v3d;
		}

		public function get invInertiaLocal() : Vector3D {
			return m_invInertiaLocal.v3d;
		}

		public function get totalForce() : Vector3D {
			return m_totalForce.v3d;
		}

		public function get totalTorque() : Vector3D {
			return m_totalTorque.sv3d;
		}

		public function get mass() : Number {
			return 1 / inverseMass;
		}

		public function get inverseMass() : Number {
			return memUser._mrf(pointer + 336);
		}

		public function set inverseMass(v : Number) : void {
			memUser._mwf(pointer + 336, v);
		}

		public function get linearDamping() : Number {
			return memUser._mrf(pointer + 436);
		}

		public function set linearDamping(v : Number) : void {
			memUser._mwf(pointer + 436, v);
		}

		public function get angularDamping() : Number {
			return memUser._mrf(pointer + 440);
		}

		public function set angularDamping(v : Number) : void {
			memUser._mwf(pointer + 440, v);
		}

		public function get linearSleepingThreshold() : Number {
			return memUser._mrf(pointer + 464) * _scaling;
		}

		public function set linearSleepingThreshold(v : Number) : void {
			memUser._mwf(pointer + 464, v / _scaling);
		}

		public function get angularSleepingThreshold() : Number {
			return memUser._mrf(pointer + 468);
		}

		public function set angularSleepingThreshold(v : Number) : void {
			memUser._mwf(pointer + 468, v);
		}

		public function get rigidbodyFlags() : int {
			return memUser._mr32(pointer + 496);
		}

		public function set rigidbodyFlags(v : int) : void {
			memUser._mw32(pointer + 496, v);
		}
	}
}
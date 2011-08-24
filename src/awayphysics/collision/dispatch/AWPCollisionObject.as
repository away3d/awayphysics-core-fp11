package awayphysics.collision.dispatch
{
	import awayphysics.AWPBase;
	import awayphysics.collision.shapes.AWPShape;
	import awayphysics.data.AWPCollisionFlags;
	import awayphysics.events.AWPCollisionEvent;
	import awayphysics.math.AWPTransform;
	import awayphysics.math.AWPVector3;
	import awayphysics.plugin.IMesh3D;

	import flash.events.Event;
	import flash.events.EventDispatcher;
	import flash.events.IEventDispatcher;
	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;

	public class AWPCollisionObject extends AWPBase implements IEventDispatcher
	{
		public static var ACTIVE_TAG : int = 1;
		public static var ISLAND_SLEEPING : int = 2;
		public static var WANTS_DEACTIVATION : int = 3;
		public static var DISABLE_DEACTIVATION : int = 4;
		public static var DISABLE_SIMULATION : int = 5;
		
		private var m_shape : AWPShape;
		private var m_skin : IMesh3D;
		private var m_worldTransform : AWPTransform;
		private var m_anisotropicFriction : AWPVector3;
		private var dispatcher : EventDispatcher;

		private var pos : Vector3D;
		private var rot : Vector.<Number>;
		private var tr : Matrix3D = new Matrix3D();
		private var raw : Vector.<Number> = new Vector.<Number>(16);

		public function AWPCollisionObject(ptr : uint, shape : AWPShape, skin : IMesh3D)
		{
			m_shape = shape;
			m_skin = skin;

			pointer = ptr;

			m_worldTransform = new AWPTransform(ptr + 4);
			m_anisotropicFriction = new AWPVector3(ptr + 164);

			dispatcher = new EventDispatcher(this);
		}

		public function get shape() : AWPShape {
			return m_shape;
		}

		public function get skin() : IMesh3D {
			return m_skin;
		}

		/**
		 * update the transform of skin mesh
		 * called by dynamicsWorld
		 */
		public function updateTransform() : void
		{
			pos = this.position;
			rot = this.rotation.rawData;
			tr.identity();
			raw[uint(0)] = rot[0];
			raw[uint(1)] = rot[1];
			raw[uint(2)] = rot[2];
			raw[uint(3)] = rot[3];
			raw[uint(4)] = rot[4];
			raw[uint(5)] = rot[5];
			raw[uint(6)] = rot[6];
			raw[uint(7)] = rot[7];
			raw[uint(8)] = rot[8];
			raw[uint(9)] = rot[9];
			raw[uint(10)] = rot[10];
			raw[uint(11)] = rot[11];
			raw[uint(12)] = pos.x;
			raw[uint(13)] = pos.y;
			raw[uint(14)] = pos.z;
			raw[uint(15)] = 1;
			tr.copyRawDataFrom(raw);

			if (m_skin) {
				m_skin.transform = tr;
			}
		}

		/**
		 * set the position in world coordinates
		 */
		public function set position(pos : Vector3D) : void {
			m_worldTransform.position.sv3d = pos;
		}

		public function get position() : Vector3D {
			return m_worldTransform.position.sv3d;
		}

		/**
		 * set the orientation in world coordinates
		 */
		public function set rotation(rot : Matrix3D) : void {
			m_worldTransform.rotation.m3d = rot;
		}

		public function get rotation() : Matrix3D {
			return m_worldTransform.rotation.m3d;
		}
		
		/**
		* set the position and orientation in world coordinates
		*/
		public function setWorldTransform(pos : Vector3D, rot : Matrix3D) : void
		{
			m_worldTransform.position.sv3d = pos;
			m_worldTransform.rotation.m3d = rot;
		}

		public function get anisotropicFriction() : Vector3D {
			return m_anisotropicFriction.v3d;
		}

		public function set anisotropicFriction(v : Vector3D) : void {
			m_anisotropicFriction.v3d = v;
			hasAnisotropicFriction = (v.x != 1 || v.y != 1 || v.z != 1) ? 1 : 0;
		}

		public function get friction() : Number {
			return memUser._mrf(pointer + 224);
		}

		public function set friction(v : Number) : void {
			memUser._mwf(pointer + 224, v);
		}

		public function get restitution() : Number {
			return memUser._mrf(pointer + 228);
		}

		public function set restitution(v : Number) : void {
			memUser._mwf(pointer + 228, v);
		}

		public function get hasAnisotropicFriction() : int {
			return memUser._mr32(pointer + 180);
		}

		public function set hasAnisotropicFriction(v : int) : void {
			memUser._mw32(pointer + 180, v);
		}

		public function get contactProcessingThreshold() : Number {
			return memUser._mrf(pointer + 184);
		}

		public function set contactProcessingThreshold(v : Number) : void {
			memUser._mwf(pointer + 184, v);
		}

		public function get collisionFlags() : int {
			return memUser._mr32(pointer + 204);
		}

		public function set collisionFlags(v : int) : void {
			memUser._mw32(pointer + 204, v);
		}

		public function get islandTag() : int {
			return memUser._mr32(pointer + 208);
		}

		public function set islandTag(v : int) : void {
			memUser._mw32(pointer + 208, v);
		}

		public function get companionId() : int {
			return memUser._mr32(pointer + 212);
		}

		public function set companionId(v : int) : void {
			memUser._mw32(pointer + 212, v);
		}

		public function get deactivationTime() : Number {
			return memUser._mrf(pointer + 220);
		}

		public function set deactivationTime(v : Number) : void {
			memUser._mwf(pointer + 220, v);
		}

		public function get activationState() : int {
			return memUser._mr32(pointer + 216);
		}

		public function set activationState(newState : int) : void {
			if (activationState != AWPCollisionObject.DISABLE_DEACTIVATION && activationState != AWPCollisionObject.DISABLE_SIMULATION) {
				memUser._mw32(pointer + 216, newState);
			}
		}

		public function forceActivationState(newState : int) : void
		{
			memUser._mw32(pointer + 216, newState);
		}

		public function activate(forceActivation : Boolean = false) : void
		{
			if (forceActivation || (collisionFlags != AWPCollisionFlags.CF_STATIC_OBJECT && collisionFlags != AWPCollisionFlags.CF_KINEMATIC_OBJECT)) {
				this.activationState = AWPCollisionObject.ACTIVE_TAG;
				this.deactivationTime = 0;
			}
		}

		public function get isActive() : Boolean {
			return (activationState != AWPCollisionObject.ISLAND_SLEEPING && activationState != AWPCollisionObject.DISABLE_SIMULATION);
		}

		public function addEventListener(type : String, listener : Function, useCapture : Boolean = false, priority : int = 0, useWeakReference : Boolean = false) : void
		{
			this.collisionFlags |= AWPCollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK;
			dispatcher.addEventListener(type, listener, useCapture, priority);
		}

		public function dispatchEvent(evt : Event) : Boolean
		{
			return dispatcher.dispatchEvent(evt);
		}

		public function hasEventListener(type : String) : Boolean
		{
			return dispatcher.hasEventListener(type);
		}

		public function removeEventListener(type : String, listener : Function, useCapture : Boolean = false) : void
		{
			this.collisionFlags &= (~AWPCollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK);
			dispatcher.removeEventListener(type, listener, useCapture);
		}

		public function willTrigger(type : String) : Boolean
		{
			return dispatcher.willTrigger(type);
		}

		/**
		 * this function just called by alchemy
		 */
		public function collisionCallback(mpt : uint, body : AWPCollisionObject) : void
		{
			var pt : AWPManifoldPoint = new AWPManifoldPoint(mpt);
			var event : AWPCollisionEvent = new AWPCollisionEvent(AWPCollisionEvent.COLLISION_ADDED);
			event.manifoldPoint = pt;
			event.collisionObject = body;

			this.dispatchEvent(event);
		}
	}
}
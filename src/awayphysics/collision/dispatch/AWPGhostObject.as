package awayphysics.collision.dispatch {
	import away3d.containers.ObjectContainer3D;

	import awayphysics.collision.shapes.AWPShape;

	/**
	 *used for create the character controller
	 */
	public class AWPGhostObject extends AWPCollisionObject {
		public function AWPGhostObject(shape : AWPShape, skin : ObjectContainer3D = null) {
			pointer = bullet.createGhostObjectMethod(this, shape.pointer);
			super(pointer, shape, skin);
		}
	}
}
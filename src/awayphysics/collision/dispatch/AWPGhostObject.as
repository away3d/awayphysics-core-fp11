package awayphysics.collision.dispatch
{
	import awayphysics.collision.shapes.AWPShape;
	import awayphysics.plugin.IMesh3D;

	/**
	 *used for create the character controller
	 */
	public class AWPGhostObject extends AWPCollisionObject
	{
		public function AWPGhostObject(shape : AWPShape, skin : IMesh3D = null)
		{
			pointer = bullet.createGhostObjectMethod(this, shape.pointer);
			super(pointer, shape, skin);
		}
	}
}
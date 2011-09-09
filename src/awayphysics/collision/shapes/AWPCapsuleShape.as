package awayphysics.collision.shapes {
	public class AWPCapsuleShape extends AWPCollisionShape {
		public function AWPCapsuleShape(radius : Number = 50, height : Number = 100) {
			pointer = bullet.createCapsuleShapeMethod(radius / _scaling, height / _scaling);
			super(pointer, 3);
		}
	}
}
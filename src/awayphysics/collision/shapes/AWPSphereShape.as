package awayphysics.collision.shapes {
	public class AWPSphereShape extends AWPCollisionShape {
		public function AWPSphereShape(radius : Number = 50) {
			pointer = bullet.createSphereShapeMethod(radius / _scaling);
			super(pointer, 1);
		}
	}
}
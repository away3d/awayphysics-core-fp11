package awayphysics.collision.shapes {
	public class AWPConeShape extends AWPCollisionShape {
		public function AWPConeShape(radius : Number = 50, height : Number = 100) {
			pointer = bullet.createConeShapeMethod(radius / _scaling, height / _scaling);
			super(pointer, 4);
		}
	}
}
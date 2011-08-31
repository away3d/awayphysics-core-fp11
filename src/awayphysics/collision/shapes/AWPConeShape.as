package awayphysics.collision.shapes {
	public class AWPConeShape extends AWPShape {
		public function AWPConeShape(radius : Number = 50, height : Number = 100) {
			pointer = bullet.createConeShapeMethod(radius / _scaling, height / _scaling);
		}
	}
}
package awayphysics.collision.shapes {
	public class AWPBoxShape extends AWPShape {
		public function AWPBoxShape(width : Number = 100, height : Number = 100, depth : Number = 100) {
			pointer = bullet.createBoxShapeMethod(width / _scaling, height / _scaling, depth / _scaling);
		}
	}
}
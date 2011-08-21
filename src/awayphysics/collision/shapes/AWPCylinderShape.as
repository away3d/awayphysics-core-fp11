package awayphysics.collision.shapes
{
	public class AWPCylinderShape extends AWPShape
	{
		public function AWPCylinderShape(radius : Number = 50, height : Number = 100)
		{
			pointer = bullet.createCylinderShapeMethod(radius * 2 / _scaling, height / _scaling, radius * 2 / _scaling);
		}
	}
}
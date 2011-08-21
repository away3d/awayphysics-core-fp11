package awayphysics.collision.shapes
{
	public class AWPSphereShape extends AWPShape
	{
		public function AWPSphereShape(radius : Number = 50)
		{
			pointer = bullet.createSphereShapeMethod(radius / _scaling);
		}
	}
}
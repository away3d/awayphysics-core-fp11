package awayphysics.collision.shapes
{
	public class AWPCapsuleShape extends AWPShape
	{
		public function AWPCapsuleShape(radius : Number = 50, height : Number = 100)
		{
			pointer = bullet.createCapsuleShapeMethod(radius / _scaling, height / _scaling);
		}
	}
}
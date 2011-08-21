package awayphysics.collision.shapes
{
	import flash.geom.Vector3D;

	public class AWPStaticPlaneShape extends AWPShape
	{
		public function AWPStaticPlaneShape(normal : Vector3D = null, constant : Number = 0)
		{
			if (!normal) {
				normal = new Vector3D(0, 1, 0);
			}
			pointer = bullet.createStaticPlaneShapeMethod(normal.x, normal.y, normal.z, constant / _scaling);
		}
	}
}
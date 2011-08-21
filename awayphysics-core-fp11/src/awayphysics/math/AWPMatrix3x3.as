package awayphysics.math
{
	import awayphysics.AWPBase;

	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;

	public class AWPMatrix3x3 extends AWPBase
	{
		public var col1 : AWPVector3;
		public var col2 : AWPVector3;
		public var col3 : AWPVector3;

		public function AWPMatrix3x3(ptr : uint)
		{
			pointer = ptr;
			col1 = new AWPVector3(ptr + 0);
			col2 = new AWPVector3(ptr + 16);
			col3 = new AWPVector3(ptr + 32);
		}

		public function get m3d() : Matrix3D {
			return new Matrix3D(Vector.<Number>([col1.x, col2.x, col3.x, 0, col1.y, col2.y, col3.y, 0, col1.z, col2.z, col3.z, 0, 0, 0, 0, 1]));
		}

		public function set m3d(m : Matrix3D) : void {
			col1.v3d = new Vector3D(m.rawData[0], m.rawData[4], m.rawData[8]);
			col2.v3d = new Vector3D(m.rawData[1], m.rawData[5], m.rawData[9]);
			col3.v3d = new Vector3D(m.rawData[2], m.rawData[6], m.rawData[10]);
		}
	}
}
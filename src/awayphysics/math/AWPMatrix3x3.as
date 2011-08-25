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
		
		private var _m3d:Matrix3D = new Matrix3D();
		private var _v3d:Vector3D = new Vector3D();

		public function AWPMatrix3x3(ptr : uint)
		{
			pointer = ptr;
			col1 = new AWPVector3(ptr + 0);
			col2 = new AWPVector3(ptr + 16);
			col3 = new AWPVector3(ptr + 32);
		}

		public function get m3d() : Matrix3D {
			_m3d.copyColumnFrom(0, col1.v3d);
			_m3d.copyColumnFrom(1, col2.v3d);
			_m3d.copyColumnFrom(2, col3.v3d);
			return _m3d;
		}

		public function set m3d(m : Matrix3D) : void {
			_v3d.setTo(m.rawData[0], m.rawData[4], m.rawData[8]);
			col1.v3d = _v3d;
			_v3d.setTo(m.rawData[1], m.rawData[5], m.rawData[9]);
			col2.v3d = _v3d;
			_v3d.setTo(m.rawData[2], m.rawData[6], m.rawData[10]);
			col3.v3d = _v3d;
		}
		
		public function getRow(row:uint):Vector3D {
			if (row == 0) {
				_v3d.setTo(col1.x, col2.x, col3.x);
			}else if (row == 1) {
				_v3d.setTo(col1.y, col2.y, col3.y);
			}else {
				_v3d.setTo(col1.z, col2.z, col3.z);
			}
			return _v3d;
		}
	}
}
package awayphysics.math {
	import awayphysics.AWPBase;

	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;

	public class AWPMatrix3x3 extends AWPBase {
		public var row1 : AWPVector3;
		public var row2 : AWPVector3;
		public var row3 : AWPVector3;
		private var _m3d : Matrix3D = new Matrix3D();
		private var _v3d : Vector3D = new Vector3D();

		public function AWPMatrix3x3(ptr : uint) {
			pointer = ptr;
			row1 = new AWPVector3(ptr + 0);
			row2 = new AWPVector3(ptr + 16);
			row3 = new AWPVector3(ptr + 32);
		}

		public function get m3d() : Matrix3D {
			_m3d.copyRowFrom(0, row1.v3d);
			_m3d.copyRowFrom(1, row2.v3d);
			_m3d.copyRowFrom(2, row3.v3d);
			return _m3d;
		}

		public function set m3d(m : Matrix3D) : void {
			_v3d.setTo(m.rawData[0], m.rawData[4], m.rawData[8]);
			row1.v3d = _v3d;
			_v3d.setTo(m.rawData[1], m.rawData[5], m.rawData[9]);
			row2.v3d = _v3d;
			_v3d.setTo(m.rawData[2], m.rawData[6], m.rawData[10]);
			row3.v3d = _v3d;
		}
	}
}
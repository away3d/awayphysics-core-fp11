package awayphysics.math {
	
	import flash.geom.Vector3D;
	import flash.geom.Matrix3D;
	
	import awayphysics.AWPBase;

	public class AWPTransform extends AWPBase {
		private var m_basis : AWPMatrix3x3;
		private var m_origin : AWPVector3;
		
		private var _transData:Vector.<Vector3D>;
		private var _transform:Matrix3D = new Matrix3D();

		public function AWPTransform(ptr : uint = 0) {
			pointer = ptr;
			
			_transData = new Vector.<Vector3D>(3, true);
			
			if (ptr > 0) {	
				m_basis = new AWPMatrix3x3(ptr + 0);
				m_origin = new AWPVector3(ptr + 48);
				
				_transData[0] = m_origin.sv3d;
				_transData[1] = AWPMath.matrix2euler(m_basis.m3d);
				_transData[2] = new Vector3D(1, 1, 1);
			}else {
				m_basis = null;
				m_origin = null;
				
				_transData[0] = new Vector3D();
				_transData[1] = new Vector3D();
				_transData[2] = new Vector3D(1, 1, 1);
			}
		}
		
		public function get origin():AWPVector3 {
			return m_origin;
		}
		
		public function get basis():AWPMatrix3x3 {
			return m_basis;
		}
		
		public function get position():Vector3D {
			if (m_origin) {
				return m_origin.sv3d;
			}else {
				return _transData[0];
			}
		}
		
		public function set position(v:Vector3D):void {
			_transData[0] = v;
			if ((m_origin)) {
				m_origin.sv3d = v;
			}
		}
		
		public function get rotation():Vector3D {
			if (m_basis) {
				return AWPMath.matrix2euler(m_basis.m3d);
			}else {
				return _transData[1];
			}
		}
		
		public function set rotation(v:Vector3D):void {
			_transData[1] = v;
			if (m_basis) {
				m_basis.m3d = AWPMath.euler2matrix(v);
			}
		}
		
		public function get transform():Matrix3D {
			if (m_origin && m_basis) {
				_transData[0] = m_origin.sv3d;
				_transData[1] = AWPMath.matrix2euler(m_basis.m3d);
			}
			_transform.identity();
			_transform.recompose(_transData);
			
			return _transform;
		}
		
		public function set transform(m:Matrix3D):void {
			_transData = m.decompose();
			_transData[2].setTo(1, 1, 1);
			if (m_origin && m_basis) {
				m_origin.sv3d = _transData[0];
				m_basis.m3d = AWPMath.euler2matrix(_transData[1]);
			}
		}
	}
}
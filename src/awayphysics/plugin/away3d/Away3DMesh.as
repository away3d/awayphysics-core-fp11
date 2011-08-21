package awayphysics.plugin.away3d
{
	import away3d.entities.Mesh;

	import awayphysics.plugin.IMesh3D;

	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;

	public class Away3DMesh implements IMesh3D
	{
		private var _mesh : Mesh;
		private var _translationOffset : Vector3D;
		private var _scale : Vector3D;

		public function Away3DMesh(do3d : Mesh, offset : Vector3D = null)
		{
			this._mesh = do3d;

			if (offset) {
				_translationOffset = offset.clone();
			}
			if (do3d.scaleX != 1 || do3d.scaleY != 1 || do3d.scaleZ != 1) {
				_scale = new Vector3D(do3d.scaleX, do3d.scaleY, do3d.scaleZ);
			}
		}

		public function get transform() : Matrix3D {
			return _mesh.transform;
		}

		public function set transform(m : Matrix3D) : void {
			var tr : Matrix3D = new Matrix3D();
			if (_translationOffset) tr.appendTranslation(_translationOffset.x, _translationOffset.y, _translationOffset.z);
			if (_scale) tr.appendScale(_scale.x, _scale.y, _scale.z);
			tr.append(m);

			_mesh.transform = tr;
		}

		public function get mesh() : Mesh {
			return _mesh;
		}

		public function get vertices() : Vector.<Number> {
			return _mesh.geometry.subGeometries[0].vertexData;
		}

		public function get indices() : Vector.<uint> {
			return _mesh.geometry.subGeometries[0].indexData;
		}
	}
}
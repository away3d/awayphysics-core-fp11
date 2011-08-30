package awayphysics.collision.shapes {
	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;

	public class AWPCompoundShape extends AWPShape {
		private var m_children : Vector.<AWPShape>;

		/**
		 *create a compound shape use the other primitive shapes
		 */
		public function AWPCompoundShape() {
			pointer = bullet.createCompoundShapeMethod();

			m_children = new Vector.<AWPShape>();
		}

		/**
		 *add a child shape and set its position and rotation in local coordinates
		 */
		public function addChildShape(child : AWPShape, localPos : Vector3D, localRot : Matrix3D) : void {
			var rotArr : Vector.<Number> = localRot.rawData;
			bullet.addCompoundChildMethod(pointer, child.pointer, localPos.x / _scaling, localPos.y / _scaling, localPos.z / _scaling, rotArr[0], rotArr[4], rotArr[8], rotArr[1], rotArr[5], rotArr[9], rotArr[2], rotArr[6], rotArr[10]);

			m_children.push(child);
		}

		/**
		 *remove a child shape from compound shape
		 */
		public function removeChildShapeByIndex(childShapeindex : int) : void {
			bullet.removeCompoundChildMethod(pointer, childShapeindex);

			m_children.splice(childShapeindex, 1);
		}

		/**
		 *get the children list
		 */
		public function get children() : Vector.<AWPShape> {
			return m_children;
		}
	}
}
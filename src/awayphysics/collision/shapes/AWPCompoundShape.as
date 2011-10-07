package awayphysics.collision.shapes {
	import awayphysics.math.AWPMath;
	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;

	public class AWPCompoundShape extends AWPCollisionShape {
		private var m_children : Vector.<AWPCollisionShape>;

		/**
		 *create a compound shape use the other primitive shapes
		 */
		public function AWPCompoundShape() {
			pointer = bullet.createCompoundShapeMethod();
			super(pointer, 7);
			m_children = new Vector.<AWPCollisionShape>();
		}

		/**
		 *add a child shape and set its position and rotation in local coordinates
		 */
		public function addChildShape(child : AWPCollisionShape, localPos : Vector3D = null, localRot : Vector3D = null) : void {
			
			if ( localPos == null )
			localPos = new Vector3D();
			
			if ( localRot == null )
			localRot = new Vector3D();
			
			var rot:Matrix3D = AWPMath.euler2matrix(AWPMath.degrees2radiansV3D(localRot));
			var rotArr : Vector.<Number> = rot.rawData;
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
		public function get children() : Vector.<AWPCollisionShape> {
			return m_children;
		}
	}
}
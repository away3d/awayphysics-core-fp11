package awayphysics.collision.shapes {
	import flash.geom.Vector3D;
	import awayphysics.AWPBase;
	
	public class AWPCollisionShape extends AWPBase {
		
		private var m_shapeType:int;
		
		public function AWPCollisionShape(ptr:uint, type:int) {
			pointer = ptr;
			m_shapeType = type;
		}
		
		/**
		 * the values defined by AWPCollisionShapeType
		 */
		public function get shapeType():int {
			return m_shapeType;
		}
		
		public function setLocalScaling(scaleX:Number, scaleY:Number, scaleZ:Number):void {
			bullet.setShapeScalingMethod(pointer, scaleX, scaleY, scaleZ);
		}
	}
}
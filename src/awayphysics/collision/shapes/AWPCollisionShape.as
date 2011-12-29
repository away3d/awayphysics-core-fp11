package awayphysics.collision.shapes {
	import flash.geom.Vector3D;
	import awayphysics.AWPBase;
	
	public class AWPCollisionShape extends AWPBase {
		
		protected var m_shapeType:int;
		protected var m_localScaling:Vector3D;
		
		public function AWPCollisionShape(ptr:uint, type:int) {
			pointer = ptr;
			m_shapeType = type;
			
			m_localScaling = new Vector3D(1, 1, 1);
		}
		
		/**
		 * the values defined by AWPCollisionShapeType
		 */
		public function get shapeType():int {
			return m_shapeType;
		}
		
		public function get localScaling():Vector3D {
			return m_localScaling;
		}
		
		public function set localScaling(scale:Vector3D):void {
			m_localScaling.setTo(scale.x, scale.y, scale.z);
			if(scale.w == 0)
				bullet.setShapeScalingMethod(pointer, scale.x, scale.y, scale.z);
		}
	}
}
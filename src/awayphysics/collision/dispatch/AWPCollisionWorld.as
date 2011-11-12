package awayphysics.collision.dispatch {
	import awayphysics.AWPBase;
	import awayphysics.collision.dispatch.AWPCollisionObject;
	
	public class AWPCollisionWorld extends AWPBase{
		
		protected var m_collisionObjects : Vector.<AWPCollisionObject>;
		
		public function AWPCollisionWorld(){
			m_collisionObjects =  new Vector.<AWPCollisionObject>();
		}
		
		public function get collisionObjects() : Vector.<AWPCollisionObject> {
			return m_collisionObjects;
		}
		
		public function addCollisionObject(obj:AWPCollisionObject, group:int = 1, mask:int = -1):void{
			bullet.addCollisionObjectMethod(obj.pointer, group, mask);
			
			if(m_collisionObjects.indexOf(obj) < 0){
				m_collisionObjects.push(obj);
			}
		}
		
		public function removeCollisionObject(obj:AWPCollisionObject) : void {
			bullet.removeCollisionObjectMethod(obj.pointer);
			
			if(m_collisionObjects.indexOf(obj) >= 0) {
				m_collisionObjects.splice(m_collisionObjects.indexOf(obj), 1);
			}
		}
	}
}
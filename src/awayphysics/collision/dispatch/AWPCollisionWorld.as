package awayphysics.collision.dispatch {
	import awayphysics.AWPBase;
	import awayphysics.collision.dispatch.AWPCollisionObject;
	import awayphysics.collision.shapes.AWPBvhTriangleMeshShape;
	import awayphysics.collision.shapes.AWPConvexHullShape;
	import awayphysics.collision.shapes.AWPHeightfieldTerrainShape;
	import awayphysics.collision.shapes.AWPCompoundShape;
	import awayphysics.data.AWPCollisionShapeType;
	
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
			obj.removeAllRays();
			if(obj.shape.shapeType==AWPCollisionShapeType.TRIANGLE_MESH_SHAPE){
				AWPBvhTriangleMeshShape(obj.shape).deleteBvhTriangleMeshShapeBuffer();
			}else if(obj.shape.shapeType==AWPCollisionShapeType.CONVEX_HULL_SHAPE){
				AWPConvexHullShape(obj.shape).deleteConvexHullShapeBuffer();
			}else if(obj.shape.shapeType==AWPCollisionShapeType.HEIGHT_FIELD_TERRAIN){
				AWPHeightfieldTerrainShape(obj.shape).deleteHeightfieldTerrainShapeBuffer();
			}else if(obj.shape.shapeType==AWPCollisionShapeType.COMPOUND_SHAPE){
				AWPCompoundShape(obj.shape).removeAllChildren();
			}
			bullet.removeCollisionObjectMethod(obj.pointer);
			
			if(m_collisionObjects.indexOf(obj) >= 0) {
				m_collisionObjects.splice(m_collisionObjects.indexOf(obj), 1);
			}
		}
	}
}
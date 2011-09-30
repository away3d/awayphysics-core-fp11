package awayphysics.collision.shapes
{
	import away3d.core.base.Geometry;
	
	public class AWPConvexHullShape extends AWPCollisionShape
	{
		private var vertexDataPtr : uint;
		
		public function AWPConvexHullShape(geometry : Geometry)
		{
			var vertexData : Vector.<Number> = geometry.subGeometries[0].vertexData;
			var vertexDataLen : int = vertexData.length;
			vertexDataPtr = bullet.createTriangleVertexDataBufferMethod(vertexDataLen);
			
			alchemyMemory.position = vertexDataPtr;
			for (var i:int = 0; i < vertexDataLen; i++ ) {
				alchemyMemory.writeFloat(vertexData[i] / _scaling);
			}
			
			pointer = bullet.createConvexHullShapeMethod(int(vertexDataLen / 3), vertexDataPtr);
			super(pointer, 5);
		}
		
		public function deleteConvexHullShapeBuffer() : void
		{
			bullet.removeTriangleVertexDataBufferMethod(vertexDataPtr);
		}
	}
}
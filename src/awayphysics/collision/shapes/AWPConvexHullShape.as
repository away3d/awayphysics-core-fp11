package awayphysics.collision.shapes
{
	import away3d.entities.Mesh;
	
	public class AWPConvexHullShape extends AWPShape
	{
		private var vertexDataPtr : uint;
		
		public function AWPConvexHullShape(mesh:Mesh)
		{
			var vertexData : Vector.<Number> = mesh.geometry.subGeometries[0].vertexData;
			var vertexDataLen : int = vertexData.length;
			vertexDataPtr = bullet.createTriangleVertexDataBufferMethod(vertexDataLen);
			
			alchemyMemory.position = vertexDataPtr;
			for (var i:int = 0; i < vertexDataLen; i++ ) {
				alchemyMemory.writeFloat(vertexData[i] / _scaling);
			}
			
			pointer = bullet.createConvexHullShapeMethod(int(vertexDataLen / 3),vertexDataPtr);
		}
		
		public function deleteConvexHullShapeBuffer() : void
		{
			bullet.removeTriangleVertexDataBufferMethod(vertexDataPtr);
		}
	}
}
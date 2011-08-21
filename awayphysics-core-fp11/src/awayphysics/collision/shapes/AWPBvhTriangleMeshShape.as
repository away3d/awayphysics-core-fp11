package awayphysics.collision.shapes
{
	import awayphysics.plugin.IMesh3D;

	public class AWPBvhTriangleMeshShape extends AWPShape
	{
		private var indexDataPtr : uint;
		private var vertexDataPtr : uint;

		public function AWPBvhTriangleMeshShape(mesh : IMesh3D, useQuantizedAabbCompression : Boolean = true)
		{
			var indexData : Vector.<uint> = mesh.indices;
			var indexDataLen : int = indexData.length;
			indexDataPtr = bullet.createTriangleIndexDataBufferMethod(indexDataLen);

			alchemyMemory.position = indexDataPtr;
			for (var i : int = 0; i < indexDataLen; i++ ) {
				alchemyMemory.writeInt(indexData[i]);
			}

			var vertexData : Vector.<Number> = mesh.vertices;
			var vertexDataLen : int = vertexData.length;
			vertexDataPtr = bullet.createTriangleVertexDataBufferMethod(vertexDataLen);

			alchemyMemory.position = vertexDataPtr;
			for (i = 0; i < vertexDataLen; i++ ) {
				alchemyMemory.writeFloat(vertexData[i] / _scaling);
			}

			var triangleIndexVertexArrayPtr : uint = bullet.createTriangleIndexVertexArrayMethod(int(indexDataLen / 3), indexDataPtr, int(vertexDataLen / 3), vertexDataPtr);

			pointer = bullet.createBvhTriangleMeshShapeMethod(triangleIndexVertexArrayPtr, useQuantizedAabbCompression ? 1 : 0, 1);
		}

		public function deleteBvhTriangleMeshShapeBuffer() : void
		{
			bullet.removeTriangleIndexDataBufferMethod(indexDataPtr);
			bullet.removeTriangleVertexDataBufferMethod(vertexDataPtr);
		}
	}
}
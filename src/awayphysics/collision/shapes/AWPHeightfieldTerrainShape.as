package awayphysics.collision.shapes {
	import awayphysics.AWPTerrain;

	public class AWPHeightfieldTerrainShape extends AWPShape {
		private var dataPtr : uint;

		/*
		 * create terrain with the heightmap data
		 */
		public function AWPHeightfieldTerrainShape(terrain : AWPTerrain) {
			var dataLen : int = terrain.sw * terrain.sh;
			dataPtr = bullet.createHeightmapDataBufferMethod(dataLen);

			var data : Vector.<Number> = terrain.heights;
			alchemyMemory.position = dataPtr;
			for (var i : int = 0; i < dataLen; i++ ) {
				alchemyMemory.writeFloat(data[i] / _scaling);
			}

			pointer = bullet.createTerrainShapeMethod(dataPtr, terrain.sw, terrain.sh, terrain.lw / _scaling, terrain.lh / _scaling, 1, -terrain.maxHeight / _scaling, terrain.maxHeight / _scaling, 1);
		}

		/*
		 * release the heightmap data buffer
		 */
		public function deleteHeightfieldTerrainShapeBuffer() : void {
			bullet.removeHeightmapDataBufferMethod(dataPtr);
		}
	}
}
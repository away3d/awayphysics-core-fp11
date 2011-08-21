package awayphysics.math
{
	import awayphysics.AWPBase;

	public class AWPTransform extends AWPBase
	{
		public var rotation : AWPMatrix3x3;
		public var position : AWPVector3;

		public function AWPTransform(ptr : uint)
		{
			pointer = ptr;

			rotation = new AWPMatrix3x3(ptr + 0);
			position = new AWPVector3(ptr + 48);
		}
	}
}
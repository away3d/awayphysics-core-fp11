package awayphysics.plugin
{
	import flash.geom.Matrix3D;

	public interface IMesh3D
	{
		/**
		 * @return A matrix with the current transformation values of the mesh.
		 */
		function get transform() : Matrix3D;

		/**
		 * Apply a matrix to the mesh.
		 */
		function set transform(m : Matrix3D) : void;

		function get vertices() : Vector.<Number>;

		function get indices() : Vector.<uint>;
	}
}
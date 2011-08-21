package awayphysics.plugin
{
	public interface ITerrain
	{
		// Number of segments horizontally.
		function get sw() : int;

		// Number of segments vertically
		function get sh() : int;

		// horizontally length of the terrain
		function get lw() : Number;

		// vertically length of the terrain
		function get lh() : Number;

		// the max height of terrain
		function get maxHeight() : Number;

		// the heights of all vertices
		function get heights() : Vector.<Number>;
	}
}
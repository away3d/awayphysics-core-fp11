package awayphysics.data {
	public class AWPCollisionFilterGroups {
		public static var DefaultFilter : int = 1;
		public static var StaticFilter : int = 2;
		public static var KinematicFilter : int = 4;
		public static var DebrisFilter : int = 8;
		public static var SensorTrigger : int = 16;
		public static var CharacterFilter : int = 32;
		public static var AllFilter : int = -1;
		// all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger
	}
}
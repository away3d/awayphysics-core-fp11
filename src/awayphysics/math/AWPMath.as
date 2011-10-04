package awayphysics.math 
{
	import flash.geom.Matrix3D;
	import flash.geom.Vector3D;
	
	public class AWPMath 
	{
		
		public static function matrix2euler( m:Matrix3D ) : Vector3D
		{
			var euler:Vector3D = new Vector3D();
			var data:Vector.<Number> = m.rawData;
			
			var sx:Number = Math.sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
			var sy:Number = Math.sqrt(data[4] * data[4] + data[5] * data[5] + data[6] * data[6]);
			var sz:Number = Math.sqrt(data[8] * data[8] + data[9] * data[9] + data[10] * data[10]);
			
			var n11:Number = data[0] / sx;
			var n21:Number = data[1] / sy;
			var n31:Number = data[2] / sz;
			var n32:Number = data[6] / sz;
			var n33:Number = data[10] / sz;
			
			n31 = n31 > 1 ? 1 : n31;
			n31 = n31 < -1 ? -1 : n31;
			
			euler.y = Math.asin(-n31);
			euler.z = Math.atan2(n21, n11);
			euler.x = Math.atan2(n32, n33);
			
			return euler;
		}
		
		public static function euler2matrix( deg:Vector3D ) : Matrix3D
		{
			var m:Matrix3D = new Matrix3D();
			var data:Vector.<Number> = m.rawData;
			
			var ax:Number = deg.x * Math.PI / 180;
			var ay:Number = deg.y * Math.PI / 180;
			var az:Number = deg.z * Math.PI / 180;

			var a:Number = Math.cos( ax );
			var b:Number = Math.sin( ax );
			var c:Number = Math.cos( ay );
			var d:Number = Math.sin( ay );
			var e:Number = Math.cos( az );
			var f:Number = Math.sin( az );

			var ad:Number = a * d;
			var bd:Number = b * d;

			data[0] =  c  * e;
			data[4] = -c  * f;
			data[8] =  d;
			data[1] =  bd * e + a * f;
			data[5] = -bd * f + a * e;
			data[9] = -b  * c;
			data[2] = -ad * e + b * f;
			data[6] =  ad * f + b * e;
			data[10] =  a  * c;
			
			m.rawData = data;
			
			return m;
		}
		
		public static function vectorMultiply(v1:Vector3D, v2:Vector3D):Vector3D {
			return new Vector3D(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
		}
	}
}
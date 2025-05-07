using System;

namespace FlatPhysicsEngineFor2D
{
	public static class FlathMath
	{

		public static float Length(FlatVector vector)
		{
			//float deltaX = vector.X - 0f;
			//float deltaY = vector.Y - 0f;
			//return (float)Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
			return (float)Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y);
		}

		public static float Distance(FlatVector a, FlatVector b)
		{
			float deltaX = a.X - b.X;
			float deltaY = a.Y - b.Y;
			return (float)Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
		}

		public static FlatVector Normalize(FlatVector vector)
		{
			float length = Length(vector);
			//float x = vector.X / length;
			//float y = vector.Y / length;
			//return new FlatVector(x, y);
			return new FlatVector(vector.X / length, vector.Y / length);
		}

		public static float Dot(FlatVector a, FlatVector b)
		{
			//a · b = ax × bx + ay × by
			return a.X * b.X + a.Y * b.Y;
		}

		public static float Cross(FlatVector a, FlatVector b)
		{
			//cx og zy er fjernet da de ikke er relevante i 2D
			//cz = axby − aybx
			return a.X * b.Y - a.Y * b.X;
		}


	}
}

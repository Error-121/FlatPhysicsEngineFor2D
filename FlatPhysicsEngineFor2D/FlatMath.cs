using System;

namespace FlatPhysicsEngineFor2D
{
	public static class FlatMath
	{
		/// <summary>
		/// A very small amount, used for precision checks and comparisons.
		/// Equal to 1/2 a millimeter.
		/// </summary>
		public static readonly float VerySmallAmount = 0.0005f;

		public static float Clamp(float value, float min, float max)
		{
			if (min == max)
			{
				return min;
			}
			if (min > max)
			{
				throw new ArgumentException("min is greater than max");
			}
			if (value < min)
			{
				return min;
			}
			if (value > max)
			{
				return max;
			}
			return value;
		}

		public static int Clamp(int value, int min, int max)
		{
			if (min == max)
			{
				return min;
			}
			if (min > max)
			{
				throw new ArgumentException("min is greater than max");
			}
			if (value < min)
			{
				return min;
			}
			if (value > max)
			{
				return max;
			}
			return value;
		}

		public static float LengthSquared(FlatVector vector)
		{
			return vector._X * vector._X + vector._Y * vector._Y;
		}

		public static float DistanceSquared(FlatVector a, FlatVector b)
		{
			float deltaX = a._X - b._X;
			float deltaY = a._Y - b._Y;
			return deltaX * deltaX + deltaY * deltaY;
		}

		public static float Length(FlatVector vector)
		{
			//float deltaX = vector.X - 0f;
			//float deltaY = vector.Y - 0f;
			//return (float)Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
			return (float)Math.Sqrt(vector._X * vector._X + vector._Y * vector._Y);
		}

		public static float Distance(FlatVector a, FlatVector b)
		{
			float deltaX = a._X - b._X;
			float deltaY = a._Y - b._Y;
			return (float)Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
		}

		public static FlatVector Normalize(FlatVector vector)
		{
			float length = Length(vector);
			//float x = vector.X / length;
			//float y = vector.Y / length;
			//return new FlatVector(x, y);
			return new FlatVector(vector._X / length, vector._Y / length);
		}

		public static float Dot(FlatVector a, FlatVector b)
		{
			//a · b = ax × bx + ay × by
			return a._X * b._X + a._Y * b._Y;
		}

		public static float Cross(FlatVector a, FlatVector b)
		{
			//cx og zy er fjernet da de ikke er relevante i 2D
			//cz = axby − aybx
			return a._X * b._Y - a._Y * b._X;
		}

		public static bool NearlyEqual(float a, float b)
		{
			return MathF.Abs(a - b) < FlatMath.VerySmallAmount;
		}

		public static bool NearlyEqual(FlatVector a, FlatVector b)
		{
			return FlatMath.DistanceSquared(a, b) < FlatMath.VerySmallAmount * FlatMath.VerySmallAmount;
		}
	}
}

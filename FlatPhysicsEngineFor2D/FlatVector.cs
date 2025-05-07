namespace FlatPhysicsEngineFor2D
{
    public class FlatVector
    {
        public readonly float X;
		public readonly float Y;

		public static readonly FlatVector Zero = new FlatVector(0f, 0f);

		public FlatVector(float x, float y)
		{
			X = x;
			Y = y;
		}

		public static FlatVector operator +(FlatVector a, FlatVector b)
		{
			return new FlatVector(a.X + b.X, a.Y + b.Y);
		}

		public static FlatVector operator -(FlatVector a, FlatVector b)
		{
			return new FlatVector(a.X - b.X, a.Y - b.Y);
		}

		public static FlatVector operator -(FlatVector vector)
		{
			return new FlatVector(-vector.X, -vector.Y);
		}

		public static FlatVector operator *(FlatVector vector, float scalar)
		{
			return new FlatVector(vector.X * scalar, vector.Y * scalar);
		}

		public static FlatVector operator /(FlatVector vector, float scalar)
		{
			return new FlatVector(vector.X / scalar, vector.Y / scalar);
		}

		public bool Equals(FlatVector other)
		{
			return X == other.X && Y == other.Y;
		}

		public override bool Equals(object obj)
		{
			if (obj is FlatVector other)
			{
				return Equals(other);
			}
			return false;
		}

		public override int GetHashCode()
		{
			return (this.X, this.Y).GetHashCode();
		}

		public override string ToString()
		{
			return $"(X: {this.X}, Y: {this.Y})";
		}
	}
}

namespace FlatPhysicsEngineFor2D
{
    public class FlatVector
    {
        public readonly float _X;
		public readonly float _Y;

		public static readonly FlatVector _zero = new FlatVector(0f, 0f);

		public FlatVector(float x, float y)
		{
			_X = x;
			_Y = y;
		}

		public static FlatVector operator +(FlatVector a, FlatVector b)
		{
			return new FlatVector(a._X + b._X, a._Y + b._Y);
		}

		public static FlatVector operator -(FlatVector a, FlatVector b)
		{
			return new FlatVector(a._X - b._X, a._Y - b._Y);
		}

		public static FlatVector operator -(FlatVector vector)
		{
			return new FlatVector(-vector._X, -vector._Y);
		}

		public static FlatVector operator *(FlatVector vector, float scalar)
		{
			return new FlatVector(vector._X * scalar, vector._Y * scalar);
		}

		public static FlatVector operator *(float scalar, FlatVector vector)
		{
			return new FlatVector(vector._X * scalar, vector._Y * scalar);
		}

		public static FlatVector operator /(FlatVector vector, float scalar)
		{
			return new FlatVector(vector._X / scalar, vector._Y / scalar);
		}

		internal static FlatVector Transform(FlatVector vector, FlatTransform transform)
		{
			return new FlatVector(
				transform._cos * vector._X - transform._sin * vector._Y + transform._positionX,
				transform._sin * vector._X + transform._cos * vector._Y + transform._positionY );
		}

		public bool Equals(FlatVector other)
		{
			return _X == other._X && _Y == other._Y;
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
			return (this._X, this._Y).GetHashCode();
		}

		public override string ToString()
		{
			return $"(X: {this._X}, Y: {this._Y})";
		}
	}
}

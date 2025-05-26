using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	internal readonly struct FlatTransform
	{
		public readonly float _positionX;
		public readonly float _positionY;
		public readonly float _sin;
		public readonly float _cos;

		public readonly static FlatTransform Zero = new FlatTransform(0f, 0f, 0f);

		public FlatTransform(FlatVector position, float angle)
		{
			this._positionX = position._X;
			this._positionY = position._Y;
			this._sin = MathF.Sin(angle);
			this._cos = MathF.Cos(angle);
		}

		public FlatTransform(float x, float y, float angle)
		{
			this._positionX = x;
			this._positionY = y;
			this._sin = MathF.Sin(angle);
			this._cos = MathF.Cos(angle);
		}
	}
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	internal readonly struct FlatTransform
	{
		public readonly float positionX;
		public readonly float positionY;
		public readonly float sin;
		public readonly float cos;

		public readonly static FlatTransform Zero = new FlatTransform(0f, 0f, 0f);

		public FlatTransform(FlatVector position, float angle)
		{
			this.positionX = position.X;
			this.positionY = position.Y;
			this.sin = MathF.Sin(angle);
			this.cos = MathF.Cos(angle);
		}

		public FlatTransform(float x, float y, float angle)
		{
			this.positionX = x;
			this.positionY = y;
			this.sin = MathF.Sin(angle);
			this.cos = MathF.Cos(angle);
		}
	}
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public readonly struct FlatAABB
	{
		public readonly FlatVector _min;
		public readonly FlatVector _max;

		public FlatAABB(FlatVector min, FlatVector max)
		{
			this._min = min;
			this._max = max;
		}

		public FlatAABB(float minX, float minY, float maxX, float maxY)
		{
			this._min = new FlatVector(minX, minY);
			this._max = new FlatVector(maxX, maxY);
		}
	}
}

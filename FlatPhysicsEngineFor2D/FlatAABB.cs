﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public readonly struct FlatAABB
	{
		public readonly FlatVector Min;
		public readonly FlatVector Max;

		public FlatAABB(FlatVector min, FlatVector max)
		{
			this.Min = min;
			this.Max = max;
		}

		public FlatAABB(float minX, float minY, float maxX, float maxY)
		{
			this.Min = new FlatVector(minX, minY);
			this.Max = new FlatVector(maxX, maxY);
		}
	}
}

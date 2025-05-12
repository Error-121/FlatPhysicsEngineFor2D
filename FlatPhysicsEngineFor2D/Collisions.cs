using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public static class Collisions
	{
		public static bool IntersectCircles(FlatVector centerA, float radiusA, FlatVector centerB, float radiusB, out FlatVector normal, out float depth)
		{
			normal = FlatVector.Zero;
			depth = 0f;

			float distance = FlatMath.Distance(centerA, centerB);
			float radii = radiusA + radiusB; // radii er den samlede radius af de to cirkler

			if (distance >= radii) 
			{
				return false; // ingen overlap
			}
			else
			{
				// beregn normal og dybde
				normal = FlatMath.Normalize(centerB - centerA);
				depth = radii - distance;
				return true; // overlap
			}
		}
	}
}

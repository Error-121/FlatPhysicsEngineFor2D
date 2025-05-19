using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public static class Collisions
	{
		public static bool IntersectPolygons(FlatVector[] verticesA, FlatVector[] verticesB)
		{
			for (int i = 0; i < verticesA.Length; i++)
			{
				FlatVector va = verticesA[i];
				FlatVector vb = verticesA[(i + 1) % verticesA.Length];

				FlatVector edge = vb - va;
				FlatVector axis = new FlatVector(-edge.Y, edge.X); // normalen er vinkelret på kanten

				Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
				Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

				if (maxA < minB || maxB < minA) // hvis der ikke er overlap
				{
					return false; // ingen overlap
				}
			}

			for (int i = 0; i < verticesB.Length; i++)
			{
				FlatVector va = verticesB[i];
				FlatVector vb = verticesB[(i + 1) % verticesB.Length];

				FlatVector edge = vb - va;
				FlatVector axis = new FlatVector(-edge.Y, edge.X); // normalen er vinkelret på kanten
				
				Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
				Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);
				
				if (maxA < minB || maxB < minA) // hvis der ikke er overlap
				{
					return false; // ingen overlap
				}
			}
			return true; // overlap
		}

		private static void ProjectVertices(FlatVector[] vertices, FlatVector axis, out float min, out float max)
		{
			min = float.MaxValue;
			max = float.MinValue;

			for (int i = 0; i < vertices.Length; i++)
			{
				FlatVector v = vertices[i];
				float proj = FlatMath.Dot(v, axis);

				if (proj < min) { min = proj; }
				if (proj > max) { max = proj; }
			}
		}

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

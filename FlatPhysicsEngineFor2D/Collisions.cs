using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public static class Collisions
	{
		public static bool IntersectPolygons(FlatVector[] verticesA, FlatVector[] verticesB, out FlatVector normal, out float depth)
		{
			normal = FlatVector.Zero;
			depth = float.MaxValue; // dybden er den mindste afstand mellem de to polygoner

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

				float axisDepth = MathF.Min(maxB - minA, maxA - minB); // dybden er den mindste afstand mellem de to polygoner

				if (axisDepth < depth) // hvis dybden er mindre end den mindste dybde
				{
					depth = axisDepth; // opdater dybden
					normal = axis; // opdater normal
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

				float axisDepth = MathF.Min(maxB - minA, maxA - minB); // dybden er den mindste afstand mellem de to polygoner

				if (axisDepth < depth) // hvis dybden er mindre end den mindste dybde
				{
					depth = axisDepth; // opdater dybden
					normal = axis; // opdater normal
				}
			}

			depth /= FlatMath.Length(normal); // normaliser normalen
			normal = FlatMath.Normalize(normal); // normaliser normalen

			FlatVector centerA = FindArithmeticMean(verticesA);
			FlatVector centerB = FindArithmeticMean(verticesB);

			FlatVector direction = centerB - centerA;

			if (FlatMath.Dot(normal, direction) < 0f) // hvis normalen peger væk fra den anden polygon
			{
				normal = -normal; // inverter normalen
			}

			return true; // overlap
		}

		private static FlatVector FindArithmeticMean(FlatVector[] vertices)
		{
			float sumX = 0f;
			float sumY = 0f;

			for (int i = 0; i < vertices.Length; i++)
			{
				FlatVector v = vertices[i];
				sumX += v.X;
				sumY += v.Y;
			}

			return new FlatVector(sumX / (float)vertices.Length, sumY / (float)vertices.Length);
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

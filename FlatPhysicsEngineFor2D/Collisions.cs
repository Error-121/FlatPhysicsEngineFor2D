using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public static class Collisions
	{
		public static bool IntersectAABB(FlatAABB a, FlatAABB b)
		{
			if (a._max._X <= b._min._X || b._max._X <= a._min._X || a._max._Y <= b._min._Y || b._max._Y <= a._min._Y)
			{
				return false;
			}
			return true;
		}

		public static void FindContactPoint(FlatBody bodyA, FlatBody bodyB, out FlatVector contactOne, out FlatVector contactTwo, out int contactCount)
		{
			contactOne = FlatVector._zero;
			contactTwo = FlatVector._zero;
			contactCount = 0;

			ShapeType shapeTypeA = bodyA.shapeType;
			ShapeType shapeTypeB = bodyB.shapeType;

			if (shapeTypeA is ShapeType.Box)
			{
				if (shapeTypeB is ShapeType.Box)
				{
					
				}
				else if (shapeTypeB is ShapeType.Circle)
				{
					
				}
			}
			else if (shapeTypeA is ShapeType.Circle)
			{
				if (shapeTypeB is ShapeType.Box)
				{
					
				}
				else if (shapeTypeB is ShapeType.Circle)
				{
					Collisions.FindContactPoint(bodyB.Position, bodyB._radius, bodyA.Position, out contactOne);
					contactCount = 1;
				}
			}
		}

		private static void FindContactPoint(FlatVector centerA, float radiusA, FlatVector centerB, out FlatVector contactPoint)
		{
			FlatVector ab = FlatMath.Normalize(centerB - centerA);
			contactPoint = centerA + ab * radiusA;
		}

		public static bool Collide(FlatBody bodyA, FlatBody bodyB, out FlatVector normal, out float depth)
		{
			normal = FlatVector._zero;
			depth = 0f;

			ShapeType shapeTypeA = bodyA.shapeType;
			ShapeType shapeTypeB = bodyB.shapeType;

			if (shapeTypeA is ShapeType.Box)
			{
				if (shapeTypeB is ShapeType.Box)
				{
					return Collisions.IntersectPolygons(bodyA.Position, bodyA.GetTransformedVertices(), bodyB.Position, bodyB.GetTransformedVertices(), out normal, out depth);
				}
				else if (shapeTypeB is ShapeType.Circle)
				{
					bool result = Collisions.IntersectCirclePolygon(bodyB.Position, bodyB._radius, bodyA.Position, bodyA.GetTransformedVertices(), out normal, out depth);

					normal = -normal;
					return result;
				}
			}
			else if (shapeTypeA is ShapeType.Circle)
			{
				if (shapeTypeB is ShapeType.Box)
				{
					return Collisions.IntersectCirclePolygon(bodyA.Position, bodyA._radius, bodyB.Position, bodyB.GetTransformedVertices(), out normal, out depth);
				}
				else if (shapeTypeB is ShapeType.Circle)
				{
					return Collisions.IntersectCircles(bodyA.Position, bodyA._radius, bodyB.Position, bodyB._radius, out normal, out depth);
				}
			}

			return false;
		}

		public static bool IntersectCirclePolygon(FlatVector circleCenter, float circleRadius, FlatVector polygonCenter, FlatVector[] vertices, out FlatVector normal, out float depth)
		{
			normal = FlatVector._zero;
			depth = float.MaxValue;

			FlatVector axis = FlatVector._zero;
			float axisDepth = 0f;
			float minA, maxA, minB, maxB;

			for (int i = 0; i < vertices.Length; i++)
			{
				FlatVector va = vertices[i];
				FlatVector vb = vertices[(i + 1) % vertices.Length];

				FlatVector edge = vb - va;
				axis = new FlatVector(-edge._Y, edge._X);
				axis = FlatMath.Normalize(axis);

				Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
				Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

				if (minA >= maxB || minB >= maxA)
				{
					return false;
				}

				axisDepth = MathF.Min(maxB - minA, maxA - minB);

				if (axisDepth < depth)
				{
					depth = axisDepth;
					normal = axis;
				}
			}

			int cpIndex = Collisions.FindClosestPointOnPolygon(circleCenter, vertices);
			FlatVector cp = vertices[cpIndex];

			axis = cp - circleCenter;
			axis = FlatMath.Normalize(axis);

			Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
			Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

			if (minA >= maxB || minB >= maxA)
			{
				return false;
			}

			axisDepth = MathF.Min(maxB - minA, maxA - minB);

			if (axisDepth < depth)
			{
				depth = axisDepth;
				normal = axis;
			}

			FlatVector direction = polygonCenter - circleCenter;

			if (FlatMath.Dot(direction, normal) < 0f)
			{
				normal = -normal;
			}

			return true;
		}

		private static int FindClosestPointOnPolygon(FlatVector circleCenter, FlatVector[] vertices)
		{
			int result = -1;
			float minDistance = float.MaxValue;

			for (int i = 0; i < vertices.Length; i++)
			{
				FlatVector v = vertices[i];
				float distance = FlatMath.Distance(v, circleCenter);

				if (distance < minDistance)
				{
					minDistance = distance;
					result = i;
				}
			}

			return result;
		}

		private static void ProjectCircle(FlatVector center, float radius, FlatVector axis, out float min, out float max)
		{
			FlatVector direction = FlatMath.Normalize(axis);
			FlatVector directionAndRadius = direction * radius;

			FlatVector p1 = center + directionAndRadius;
			FlatVector p2 = center - directionAndRadius;

			min = FlatMath.Dot(p1, axis);
			max = FlatMath.Dot(p2, axis);

			if (min > max)
			{
				// swap the min and max values.
				float t = min;
				min = max;
				max = t;
			}
		}

		public static bool IntersectPolygons(FlatVector centerA, FlatVector[] verticesA, FlatVector centerB, FlatVector[] verticesB, out FlatVector normal, out float depth)
		{
			normal = FlatVector._zero;
			depth = float.MaxValue;

			for (int i = 0; i < verticesA.Length; i++)
			{
				FlatVector va = verticesA[i];
				FlatVector vb = verticesA[(i + 1) % verticesA.Length];

				FlatVector edge = vb - va;
				FlatVector axis = new FlatVector(-edge._Y, edge._X);
				axis = FlatMath.Normalize(axis);

				Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
				Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

				if (minA >= maxB || minB >= maxA)
				{
					return false;
				}

				float axisDepth = MathF.Min(maxB - minA, maxA - minB);

				if (axisDepth < depth)
				{
					depth = axisDepth;
					normal = axis;
				}
			}

			for (int i = 0; i < verticesB.Length; i++)
			{
				FlatVector va = verticesB[i];
				FlatVector vb = verticesB[(i + 1) % verticesB.Length];

				FlatVector edge = vb - va;
				FlatVector axis = new FlatVector(-edge._Y, edge._X);
				axis = FlatMath.Normalize(axis);

				Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
				Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

				if (minA >= maxB || minB >= maxA)
				{
					return false;
				}

				float axisDepth = MathF.Min(maxB - minA, maxA - minB);

				if (axisDepth < depth)
				{
					depth = axisDepth;
					normal = axis;
				}
			}

			FlatVector direction = centerB - centerA;

			if (FlatMath.Dot(direction, normal) < 0f)
			{
				normal = -normal;
			}

			return true;
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
			normal = FlatVector._zero;
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

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public sealed class FlatWorld
	{
		public static readonly float MinBodySize = 0.01f*0.01f;
		public static readonly float MaxBodySize = 64f * 64f;

		public static readonly float MinDensity = 0.05f;    // g/cm^3
		public static readonly float MaxDensity = 21.4f;    // g/cm^3

		private FlatVector gravity;
		private List<FlatBody> bodyList;

		public int BodyCount
		{
			get { return this.bodyList.Count; }
		}

		public FlatWorld()
		{
			this.gravity = new FlatVector(0f, -9.81f);
			this.bodyList = new List<FlatBody>();
		}

		public void AddBody(FlatBody body)
		{
			this.bodyList.Add(body);
		}

		public bool RemoveBody(FlatBody body)
		{
			return this.bodyList.Remove(body);
		}

		public bool GetBody(int index, out FlatBody body)
		{
			body = null;

			if (index < 0 || index >= this.bodyList.Count)
			{
				return false;
			}

			body = this.bodyList[index];
			return true;
		}

		public void Step(float time)
		{
			// Movement step
			for (int i = 0; i < this.bodyList.Count; i++)
			{
				this.bodyList[i].Step(time, this.gravity);
			}

			// collision step
			for (int i = 0; i < this.bodyList.Count - 1; i++)
			{
				FlatBody bodyA = this.bodyList[i];

				for (int j = i + 1; j < this.bodyList.Count; j++)
				{
					FlatBody bodyB = this.bodyList[j];

					if (bodyA.isStatic && bodyB.isStatic)
					{
						continue;
					}

					if (this.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
					{
						if (bodyA.isStatic)
						{
							bodyB.Move(normal * depth);
						}
						else if (bodyB.isStatic)
						{
							bodyA.Move(-normal * depth);
						}
						else
						{
							bodyA.Move(-normal * depth / 2f);
							bodyB.Move(normal * depth / 2f);
						}

						this.ResolveCollision(bodyA, bodyB, normal, depth);
					}
				}
			}
		}

		public void ResolveCollision(FlatBody bodyA, FlatBody bodyB, FlatVector normal, float depth)
		{
			FlatVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

			if (FlatMath.Dot(relativeVelocity, normal) > 0f)
			{
				return;
			}

			float e = MathF.Min(bodyA.restitution, bodyB.restitution);

			float j = -(1f + e) * FlatMath.Dot(relativeVelocity, normal);
			j /= bodyA.invMass + bodyB.invMass;

			FlatVector impulse = j * normal;

			bodyA.LinearVelocity -= impulse * bodyA.invMass;
			bodyB.LinearVelocity += impulse * bodyB.invMass;
		}

		public bool Collide(FlatBody bodyA, FlatBody bodyB, out FlatVector normal, out float depth)
		{
			normal = FlatVector.Zero;
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
					bool result = Collisions.IntersectCirclePolygon(bodyB.Position, bodyB.radius, bodyA.Position, bodyA.GetTransformedVertices(), out normal, out depth);

					normal = -normal;
					return result;
				}
			}
			else if (shapeTypeA is ShapeType.Circle)
			{
				if (shapeTypeB is ShapeType.Box)
				{
					return Collisions.IntersectCirclePolygon(bodyA.Position, bodyA.radius, bodyB.Position, bodyB.GetTransformedVertices(), out normal, out depth);
				}
				else if (shapeTypeB is ShapeType.Circle)
				{
					return Collisions.IntersectCircles(bodyA.Position, bodyA.radius, bodyB.Position, bodyB.radius, out normal, out depth);
				}
			}

			return false;
		}

	}
}

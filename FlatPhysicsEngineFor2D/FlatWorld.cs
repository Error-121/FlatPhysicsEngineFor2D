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
		public static readonly float _minBodySize = 0.01f*0.01f;
		public static readonly float _maxBodySize = 64f * 64f;

		public static readonly float _minDensity = 0.05f;    // g/cm^3
		public static readonly float _maxDensity = 21.4f;    // g/cm^3

		public static readonly int _minIterations = 1;
		public static readonly int _maxIterations = 128;

		private FlatVector _gravity;
		private List<FlatBody> _bodyList;

		public int BodyCount
		{
			get { return this._bodyList.Count; }
		}

		public FlatWorld()
		{
			this._gravity = new FlatVector(0f, -9.81f);
			this._bodyList = new List<FlatBody>();
		}

		public void AddBody(FlatBody body)
		{
			this._bodyList.Add(body);
		}

		public bool RemoveBody(FlatBody body)
		{
			return this._bodyList.Remove(body);
		}

		public bool GetBody(int index, out FlatBody body)
		{
			body = null;

			if (index < 0 || index >= this._bodyList.Count)
			{
				return false;
			}

			body = this._bodyList[index];
			return true;
		}

		public void Step(float time, int iterations)
		{
			iterations = FlatMath.Clamp(iterations, FlatWorld._minIterations, FlatWorld._maxIterations);

			for (int it = 0; it < iterations; it++)
			{


				// Movement step
				for (int i = 0; i < this._bodyList.Count; i++)
				{
					this._bodyList[i].Step(time, this._gravity, iterations);
				}

				// collision step
				for (int i = 0; i < this._bodyList.Count - 1; i++)
				{
					FlatBody bodyA = this._bodyList[i];

					for (int j = i + 1; j < this._bodyList.Count; j++)
					{
						FlatBody bodyB = this._bodyList[j];

						if (bodyA._isStatic && bodyB._isStatic)
						{
							continue;
						}

						if (this.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
						{
							if (bodyA._isStatic)
							{
								bodyB.Move(normal * depth);
							}
							else if (bodyB._isStatic)
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
		}

		public void ResolveCollision(FlatBody bodyA, FlatBody bodyB, FlatVector normal, float depth)
		{
			FlatVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

			if (FlatMath.Dot(relativeVelocity, normal) > 0f)
			{
				return;
			}

			float e = MathF.Min(bodyA._restitution, bodyB._restitution);

			float j = -(1f + e) * FlatMath.Dot(relativeVelocity, normal);
			j /= bodyA._invMass + bodyB._invMass;

			FlatVector impulse = j * normal;

			bodyA.LinearVelocity -= impulse * bodyA._invMass;
			bodyB.LinearVelocity += impulse * bodyB._invMass;
		}

		public bool Collide(FlatBody bodyA, FlatBody bodyB, out FlatVector normal, out float depth)
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

	}
}

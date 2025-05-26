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
		private List<FlatManifold> _contactList;

		public List<FlatVector> _ContactPointsList;

		public int BodyCount
		{
			get { return this._bodyList.Count; }
		}

		public FlatWorld()
		{
			this._gravity = new FlatVector(0f, -9.81f);
			this._bodyList = new List<FlatBody>();
			this._contactList = new List<FlatManifold>();

			this._ContactPointsList = new List<FlatVector>();
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

			this._ContactPointsList.Clear();

			for (int it = 0; it < iterations; it++)
			{


				// Movement step
				for (int i = 0; i < this._bodyList.Count; i++)
				{
					this._bodyList[i].Step(time, this._gravity, iterations);
				}

				// Clear contact list
				this._contactList.Clear();

				// collision step
				for (int i = 0; i < this._bodyList.Count - 1; i++)
				{
					FlatBody bodyA = this._bodyList[i];
					FlatAABB bodyA_aabb = bodyA.GetAABB();

					for (int j = i + 1; j < this._bodyList.Count; j++)
					{
						FlatBody bodyB = this._bodyList[j];
						FlatAABB bodyB_aabb = bodyB.GetAABB();

						if (bodyA._isStatic && bodyB._isStatic)
						{
							continue;
						}

						if (!Collisions.IntersectAABB(bodyA_aabb, bodyB_aabb))
						{
							continue;
						}

						if (Collisions.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
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

							Collisions.FindContactPoint(bodyA, bodyB, out FlatVector contactOne, out FlatVector contactTwo, out int contactCount);
							FlatManifold contact = new FlatManifold(bodyA, bodyB, normal, depth, contactOne, contactTwo, contactCount);
							this._contactList.Add(contact);
						}
					}
				}

				for (int i = 0; i < this._contactList.Count; i++)
				{
					FlatManifold contact = this._contactList[i];
					this.ResolveCollision(in contact);

					if (contact._contactCount > 0)
					{
						this._ContactPointsList.Add(contact._contactOne);

						if (contact._contactCount > 1)
						{
							this._ContactPointsList.Add(contact._contactTwo);
						}
					}
				}

			}
		}

		public void ResolveCollision(in FlatManifold contact)
		{
			FlatBody bodyA = contact._bodyA;
			FlatBody bodyB = contact._bodyB;
			FlatVector normal = contact._normal;
			float depth = contact._depth;

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

	}
}

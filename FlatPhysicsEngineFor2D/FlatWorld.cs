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
		private List<(int, int)> _contactPairs;


		public int BodyCount
		{
			get { return this._bodyList.Count; }
		}

		public FlatWorld()
		{
			this._gravity = new FlatVector(0f, -9.81f);
			this._bodyList = new List<FlatBody>();
			this._contactPairs = new List<(int, int)>();
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

		public void Step(float time, int totalIterations)
		{
			totalIterations = FlatMath.Clamp(totalIterations, FlatWorld._minIterations, FlatWorld._maxIterations);

			for (int currentIteration = 0; currentIteration < totalIterations; currentIteration++)
			{
				this._contactPairs.Clear();
				this.StepBodies(time, totalIterations);
				this.BroadPhaseCollision();
				this.NarrowPhaseCollision();
			}
		}

		private void BroadPhaseCollision()
		{
			// This method is typically involve checking for potential collisions
			// between bodies based on their AABBs (Axis-Aligned Bounding Boxes).
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

					this._contactPairs.Add((i, j));
				}
			}
		}

		private void NarrowPhaseCollision()
		{
			// This method typically involve checking for actual collisions
			// between bodies that were found to be potentially colliding
			// in the broad phase.

			for (int i = 0; i < this._contactPairs.Count; i++)
			{
				(int, int) pair = this._contactPairs[i];
				FlatBody bodyA = this._bodyList[pair.Item1];
				FlatBody bodyB = this._bodyList[pair.Item2];

				if (Collisions.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
				{
					this.SeparateBodies(bodyA, bodyB, normal * depth);
					Collisions.FindContactPoints(bodyA, bodyB, out FlatVector contactOne, out FlatVector contactTwo, out int contactCount);
					FlatManifold contact = new FlatManifold(bodyA, bodyB, normal, depth, contactOne, contactTwo, contactCount);
					this.ResolveCollision(in contact);
				}
			}
		}

		public void StepBodies(float time, int totalIterations) 
		{
			for (int i = 0; i < this._bodyList.Count; i++)
			{
				this._bodyList[i].Step(time, this._gravity, totalIterations);
			}
		}

		private void SeparateBodies(FlatBody bodyA, FlatBody bodyB, FlatVector mtv)
		{
			if (bodyA._isStatic)
			{
				bodyB.Move(mtv);
			}
			else if (bodyB._isStatic)
			{
				bodyA.Move(-mtv);
			}
			else
			{
				bodyA.Move(-mtv / 2f);
				bodyB.Move(mtv / 2f);
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

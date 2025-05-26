using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public readonly struct FlatManifold
	{
		public readonly FlatBody _bodyA;
		public readonly FlatBody _bodyB;
		public readonly FlatVector _normal;
		public readonly float _depth;
		public readonly FlatVector _contactOne;
		public readonly FlatVector _contactTwo;
		public readonly int _contactCount;

		public FlatManifold(FlatBody bodyA, FlatBody bodyB, FlatVector normal, float depth, FlatVector contactOne, FlatVector contactTwo, int contactCount)
		{
			_bodyA = bodyA;
			_bodyB = bodyB;
			_normal = normal;
			_depth = depth;
			_contactOne = contactOne;
			_contactTwo = contactTwo;
			_contactCount = contactCount;
		}
	}
}

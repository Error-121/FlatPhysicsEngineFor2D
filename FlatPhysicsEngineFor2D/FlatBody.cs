using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlatPhysicsEngineFor2D
{
	public enum ShapeType
	{
		Circle = 0,
		Box = 1,
	}

	public sealed class FlatBody
	{
		private FlatVector position;
		private FlatVector linearVelocity;
		private float rotation;
		private float rotationalVelocity;

		//everything below can be moved to a separate class
		public readonly float density;
		public readonly float mass;
		public readonly float restitution;
		public readonly float area;

		public readonly bool isStatic;

		public readonly float radius;
		public readonly float width;
		public readonly float height;

		public readonly ShapeType shapeType;

		public FlatVector Position 
		{
			get { return this.position; }
		}

		private FlatBody(FlatVector position, float density, float mass, float restitution, float area, bool isStatic, float radius, float width, float height, ShapeType shapeTybe) 
		{
			this.position = position;
			this.linearVelocity = FlatVector.Zero;
			this.rotation = 0f;
			this.rotationalVelocity = 0f;

			this.density = density;
			this.mass = mass;
			this.restitution = restitution;
			this.area = area;
			
			this.isStatic = isStatic;
			this.radius = radius;
			this.width = width;
			this.height = height;
			this.shapeType = shapeTybe;
		}

		public void Move(FlatVector amount) 
		{
			this.position += amount;
		}

		public void MoveTo(FlatVector position)
		{
			this.position = position;
		}

		public static bool CreateCircleBody(FlatVector position, float density, float restitution, bool isStatic, float radius, out FlatBody body, out string errorMessage)
		{
			body = null;
			errorMessage = string.Empty;

			float area = radius * radius * MathF.PI;

			if (area < FlatWorld.MinBodySize)
			{
				errorMessage = $"Circle radius is too small. Min circle area is {FlatWorld.MinBodySize}";
				return false;
			}
			if (area > FlatWorld.MaxBodySize)
			{
				errorMessage = $"Circle radius is too large. Max circle area is {FlatWorld.MaxBodySize}";
				return false;
			}

			if (density < FlatWorld.MinDensity)
			{
				errorMessage = $"Circle density is too small. Min circle density is {FlatWorld.MinDensity}";
				return false;
			}
			if (density > FlatWorld.MaxDensity)
			{
				errorMessage = $"Circle density is too large. Max circle density is {FlatWorld.MaxDensity}";
				return false;
			}

			restitution = FlatMath.Clamp(restitution, 0f, 1f);

			//mass = area * depth(Z) * density;
			//float mass = area * 1f * density;
			float mass = area * density;

			body = new FlatBody(position, density, mass, restitution, area, isStatic, radius, 0f, 0f, ShapeType.Circle);

			return true;
		}

		public static bool CreateBoxBody(FlatVector position, float density, float restitution, bool isStatic, float width, float height, out FlatBody body, out string errorMessage)
		{
			body = null;
			errorMessage = string.Empty;
			float area = width * height;
			if (area < FlatWorld.MinBodySize)
			{
				errorMessage = $"Box area is too small. Min box area is {FlatWorld.MinBodySize}";
				return false;
			}
			if (area > FlatWorld.MaxBodySize)
			{
				errorMessage = $"Box area is too large. Max box area is {FlatWorld.MaxBodySize}";
				return false;
			}
			if (density < FlatWorld.MinDensity)
			{
				errorMessage = $"Box density is too small. Min box density is {FlatWorld.MinDensity}";
				return false;
			}
			if (density > FlatWorld.MaxDensity)
			{
				errorMessage = $"Box density is too large. Max box density is {FlatWorld.MaxDensity}";
				return false;
			}
			restitution = FlatMath.Clamp(restitution, 0f, 1f);
			float mass = area * density;
			body = new FlatBody(position, density, mass, restitution, area, isStatic, 0f, width, height, ShapeType.Box);
			return true;
		}
	}
}

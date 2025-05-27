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
		private FlatVector _position;
		private FlatVector _linearVelocity;
		private float _rotation;
		private float _rotationalVelocity;

		private FlatVector _force;

		//everything below can be moved to a separate class
		public readonly float _density;
		public readonly float _mass;
		public readonly float _invMass;
		public readonly float _restitution;
		public readonly float _area;
		public readonly float _inertia;
		public readonly float _invInertia;

		public readonly bool _isStatic;

		public readonly float _radius;
		public readonly float _width;
		public readonly float _height;
		//everything above can be moved to a separate class

		private readonly FlatVector[] _vertices;
		public readonly int[] _triangles;
		private FlatVector[] _transformedVertices;
		private FlatAABB _aabb;

		private bool _transformUpdateRequired;
		private bool _aabbUpdateRequired;


		public readonly ShapeType shapeType;

		public FlatVector Position 
		{
			get { return this._position; }
		}

		public FlatVector LinearVelocity
		{
			get { return this._linearVelocity; }
			internal set { this._linearVelocity = value; }
		}

		private FlatBody(FlatVector position, float density, float mass, float restitution, float area, bool isStatic, float radius, float width, float height, ShapeType shapeTybe) 
		{
			this._position = position;
			this._linearVelocity = FlatVector._zero;
			this._rotation = 0f;
			this._rotationalVelocity = 0f;

			this._force = FlatVector._zero;

			this._density = density;
			this._mass = mass;
			this._restitution = restitution;
			this._area = area;
			

			this._isStatic = isStatic;
			this._radius = radius;
			this._width = width;
			this._height = height;
			this.shapeType = shapeTybe;
			
			this._inertia = this.CalculateRotationalInertia();


			if (!this._isStatic)
			{
				this._invMass = 1f / this._mass;
				this._invInertia = 1f / this._inertia;
			}
			else
			{
				this._invMass = 0f;
				this._invInertia = 0f;
			}

			if (this.shapeType is ShapeType.Box)
			{
				this._vertices = FlatBody.CreateBoxVertices(this._width, this._height);
				this._triangles = FlatBody.CreateBoxTriangles();
				this._transformedVertices = new FlatVector[this._vertices.Length];

			}
			else
			{
				this._vertices = null;
				_triangles = null;
				this._transformedVertices = null;

			}

			this._transformUpdateRequired = true;
			this._aabbUpdateRequired = true;
		}

		private float CalculateRotationalInertia()
		{
			if (this.shapeType is ShapeType.Circle)
			{
				// I = m * r^2 / 2
				return (1f / 2f) * this._mass * this._radius * this._radius;
			}
			else if (this.shapeType is ShapeType.Box)
			{
				// I = m * (w^2 + h^2) / 12
				return (1f / 12f) * this._mass * (this._width * this._width + this._height * this._height);
			}
			else
			{
				throw new ArgumentOutOfRangeException("ShapeType is not a valid.");
			}
		}

		private static FlatVector[] CreateBoxVertices(float width, float height)
		{
			float left = -width / 2f;
			float right = left + width;
			float bottom = -height / 2f;
			float top = bottom + height;

			FlatVector[] vertices = new FlatVector[4];
			vertices[0] = new FlatVector(left, top);
			vertices[1] = new FlatVector(right, top);
			vertices[2] = new FlatVector(right, bottom);
			vertices[3] = new FlatVector(left, bottom);

			return vertices;
		}

		private static int[] CreateBoxTriangles()
		{
			int[] triangles = new int[6];
			triangles[0] = 0;
			triangles[1] = 1;
			triangles[2] = 2;
			triangles[3] = 0;
			triangles[4] = 2;
			triangles[5] = 3;
			return triangles;
		}

		public FlatVector[] GetTransformedVertices()
		{
			if (this._transformUpdateRequired)
			{
				FlatTransform transform = new FlatTransform(this._position, this._rotation);

				for (int i = 0; i < this._vertices.Length; i++)
				{
					FlatVector v = this._vertices[i];
					this._transformedVertices[i] = FlatVector.Transform(v, transform);
				}
			}

			this._transformUpdateRequired = false;
			return this._transformedVertices;
		}

		public FlatAABB GetAABB()
		{
			if (this._aabbUpdateRequired)
			{
				float minX = float.MaxValue;
				float minY = float.MaxValue;
				float maxX = float.MinValue;
				float maxY = float.MinValue;

				if (this.shapeType is ShapeType.Box)
				{
					FlatVector[] vertices = this.GetTransformedVertices();

					for (int i = 0; i < vertices.Length; i++)
					{
						FlatVector v = vertices[i];

						if (v._X < minX) { minX = v._X; }
						if (v._X > maxX) { maxX = v._X; }
						if (v._Y < minY) { minY = v._Y; }
						if (v._Y > maxY) { maxY = v._Y; }
					}
				}
				else if (this.shapeType is ShapeType.Circle)
				{
					minX = this._position._X - this._radius;
					minY = this._position._Y - this._radius;
					maxX = this._position._X + this._radius;
					maxY = this._position._Y + this._radius;
				}
				else
				{
					throw new Exception("Unknown ShapeType.");
				}

				this._aabb = new FlatAABB(minX, minY, maxX, maxY);
			}

			this._aabbUpdateRequired = false;
			return this._aabb;
		}

		internal void Step(float time, FlatVector gravity, int iterations)
		{
			if (this._isStatic)
			{
				return;
			}

			time /= (float)iterations;

			// force = mass * acc
			// acc = force / mass;

			//FlatVector acceleration = this.force / this.mass;
			//this.linearVelocity += acceleration * time;

			this._linearVelocity += gravity * time;
			this._position += this._linearVelocity * time;

			this._rotation += this._rotationalVelocity * time;

			this._force = FlatVector._zero;
			this._transformUpdateRequired = true;
			this._aabbUpdateRequired = true;
		}

		public void Move(FlatVector amount) 
		{
			this._position += amount;
			this._transformUpdateRequired = true;
			this._aabbUpdateRequired = true;
		}

		public void MoveTo(FlatVector position)
		{
			this._position = position;
			this._transformUpdateRequired = true;
			this._aabbUpdateRequired = true;
		}

		public void Rotate(float amount)
		{
			this._rotation += amount;
			this._transformUpdateRequired = true;
			this._aabbUpdateRequired = true;
		}

		public void AddForce(FlatVector amount)
		{
			this._force = amount;
		}

		public static bool CreateCircleBody(float radius, FlatVector position, float density, bool isStatic, float restitution, out FlatBody body, out string errorMessage)
		{
			body = null;
			errorMessage = string.Empty;

			float area = radius * radius * MathF.PI;

			if (area < FlatWorld._minBodySize)
			{
				errorMessage = $"Circle radius is too small. Min circle area is {FlatWorld._minBodySize}";
				return false;
			}
			if (area > FlatWorld._maxBodySize)
			{
				errorMessage = $"Circle radius is too large. Max circle area is {FlatWorld._maxBodySize}";
				return false;
			}

			if (density < FlatWorld._minDensity)
			{
				errorMessage = $"Circle density is too small. Min circle density is {FlatWorld._minDensity}";
				return false;
			}
			if (density > FlatWorld._maxDensity)
			{
				errorMessage = $"Circle density is too large. Max circle density is {FlatWorld._maxDensity}";
				return false;
			}

			restitution = FlatMath.Clamp(restitution, 0f, 1f);

			//mass = area * depth(Z) * density;
			//float mass = area * 1f * density;
			float mass = area * density;

			body = new FlatBody(position, density, mass, restitution, area, isStatic, radius, 0f, 0f, ShapeType.Circle);

			return true;
		}

		public static bool CreateBoxBody(float width, float height, FlatVector position, float density, bool isStatic, float restitution, out FlatBody body, out string errorMessage)
		{
			body = null;
			errorMessage = string.Empty;
			float area = width * height;
			if (area < FlatWorld._minBodySize)
			{
				errorMessage = $"Box area is too small. Min box area is {FlatWorld._minBodySize}";
				return false;
			}
			if (area > FlatWorld._maxBodySize)
			{
				errorMessage = $"Box area is too large. Max box area is {FlatWorld._maxBodySize}";
				return false;
			}
			if (density < FlatWorld._minDensity)
			{
				errorMessage = $"Box density is too small. Min box density is {FlatWorld._minDensity}";
				return false;
			}
			if (density > FlatWorld._maxDensity)
			{
				errorMessage = $"Box density is too large. Max box density is {FlatWorld._maxDensity}";
				return false;
			}
			restitution = FlatMath.Clamp(restitution, 0f, 1f);
			float mass = area * density;
			body = new FlatBody(position, density, mass, restitution, area, isStatic, 0f, width, height, ShapeType.Box);
			return true;
		}
	}
}

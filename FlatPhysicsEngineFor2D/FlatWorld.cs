using System;
using System.Collections.Generic;
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
	}
}

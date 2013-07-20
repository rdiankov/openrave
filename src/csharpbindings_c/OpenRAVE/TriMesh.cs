using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace OpenRAVE
{
	/// <summary>
	/// The TriMesh class provides access to the OpenRAVE TriMesh object, define new mesh data
	/// with support for either float or doubles, depending on the OpenRAVE build.
	/// </summary>
	public class TriMesh : Base
	{
		[DllImport("libopenrave0.9_c")]
		private static extern IntPtr ORCCreateTriMesh(float[] vertices, int numvertices, float[] indices, int numtriangles);

		[DllImport("libopenrave0.9_c")]
		private static extern IntPtr ORCCreateTriMesh(double[] vertices, int numvertices, double[] indices, int numtriangles);
		
		[DllImport("libopenrave0.9_c")]
		private static extern void ORCTriMeshDestroy(IntPtr trimesh);
		
		public TriMesh(float[] vertices, float[] indices) : base(ORCCreateTriMesh(vertices, vertices.Length, indices, indices.Length)) {}		
		public TriMesh(double[] vertices, double[] indices) : base(ORCCreateTriMesh(vertices, vertices.Length, indices, indices.Length)) {}

		protected override void Destroy()
		{
			ORCTriMeshDestroy(ptr);
		}
	}	
}


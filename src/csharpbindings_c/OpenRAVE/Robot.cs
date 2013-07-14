using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace OpenRAVE
{
	/// <summary>
	/// The Robot class provides a simple wrapper around an OpenRAVE Robot.
	/// </summary>
	public class Robot : InterfaceBase
	{
		[DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
		private static extern IntPtr ORCRobotGetName(IntPtr robot);
	
		internal Robot(IntPtr ptr) : base(ptr) {}
		
		public string Name
		{
			get
			{
				return Marshal.PtrToStringAnsi(ORCRobotGetName(ptr));
			}
		}
	}
}


using System;

namespace OpenRAVE
{
	/// <summary>
	/// The Module class provides a simple wrapper around an OpenRAVE Module.
	/// </summary>
	public class Module : InterfaceBase
	{
		internal Module(IntPtr ptr) : base (ptr) {}
	}
}


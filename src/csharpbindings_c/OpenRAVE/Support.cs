using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace OpenRAVE
{
	/// <summary>
	/// The Base class provides a simple means of resource management for objects
	/// constructed and returned by OpenRAVE. The basic principle is that objects
	/// which use this as their parent class override the Destroy method to perform
	/// any destruction calls back to OpenRAVE. 
	/// </summary>
	public class Base : IDisposable
	{
		protected IntPtr ptr = IntPtr.Zero;
		internal IntPtr Ptr { get { return ptr; } }
		internal Base(IntPtr ptr) { this.ptr = ptr; }
		~Base() { Dispose(); }
		protected virtual void Destroy() {}
		public virtual void Dispose() { if(ptr != IntPtr.Zero) { Destroy(); ptr = IntPtr.Zero; } }
	}

	/// <summary>
	/// The InterfaceBase class provides automated destruction of OpenRAVE Interface
	/// based objects which used the ORCInterfaceRelease function for destruction.
	/// </summary>
	public class InterfaceBase : Base
	{
		[DllImport("libopenrave0.9_c")]
		private static extern void ORCInterfaceRelease(IntPtr pinterface);
		
		[DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
		private static extern IntPtr ORCInterfaceSendCommand(IntPtr pinterface, string command);
		
		protected InterfaceBase(IntPtr ptr) : base(ptr) {}
		
		public string SendCommand(string command)
		{
			return Marshal.PtrToStringAnsi(ORCInterfaceSendCommand(ptr, command));
		}

		protected override void Destroy()
		{
			ORCInterfaceRelease(ptr);
		}
	}

	/// <summary>
	/// The DisposableList class provides a nice simple to use collection object
	/// which has the ability to automated disposal of all of the contained components.
	/// This is particularly useful for scenarios where an array of results are
	/// returned and all need cleaning up when complete.
	/// </summary>
	public class DisposableList<T> : List<T>, IDisposable where T : IDisposable
	{
		public void Dispose()
		{
			foreach(T t in this)
				t.Dispose();
			Clear();
		}
	}
}

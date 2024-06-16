using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace OpenRAVE
{
/// <summary>
/// The Environment class provides a wrapper around the various Environment
/// related functions. An object itself must be created from the Context object.
/// </summary>
public class Environment : Base
{
    [DllImport("libopenrave0.9-core_c")]
    private static extern IntPtr ORCEnvironmentCreate();

    [DllImport("libopenrave0.9-core_c")]
    private static extern void ORCEnvironmentRelease(IntPtr pObj);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern int ORCEnvironmentLoad(IntPtr env, string filename);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCEnvironmentDestroy(IntPtr env);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern IntPtr ORCCreateKinBody(IntPtr env, string name);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern IntPtr ORCEnvironmentGetKinBody(IntPtr env, string name);

    [DllImport("libopenrave0.9_c")]
    private static extern int ORCEnvironmentGetBodies(IntPtr env, IntPtr[] bodies);

    [DllImport("libopenrave0.9_c")]
    private static extern int ORCEnvironmentGetRobots(IntPtr env, IntPtr[] robots);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCEnvironmentAdd(IntPtr env, IntPtr pinterface);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCEnvironmentRemove(IntPtr env, IntPtr pinterface);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern int ORCEnvironmentAddModule(IntPtr env, IntPtr module, string args);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern IntPtr ORCModuleCreate(IntPtr env, string modulename);

    [DllImport("libopenrave0.9_c")]
    private static extern ulong ORCEnvironmentGetSimulationTime(IntPtr env);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCEnvironmentLock(IntPtr env);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCEnvironmentUnlock(IntPtr env);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern int ORCEnvironmentSetViewer(IntPtr env, string viewername);

    internal Environment() : base(ORCEnvironmentCreate()) {
    }

    protected override void Destroy()
    {
        ORCEnvironmentDestroy(ptr);
        ORCEnvironmentRelease(ptr);
    }

    public bool Load(string filename)
    {
        if (ORCEnvironmentLoad(ptr, filename)==1)
            return true;
        else
            return false;
    }

    public Body CreateBody(string name)
    {
        return new Body(ORCCreateKinBody(ptr, name));
    }

    public Body GetBody(string name)
    {
        return new Body(ORCEnvironmentGetKinBody(ptr, name));
    }

    public int BodyCount
    {
        get
        {
            return ORCEnvironmentGetBodies(ptr, null);
        }
    }

    public DisposableList<Body> Bodies
    {
        get
        {
            IntPtr[] ptrs = new IntPtr[BodyCount];
            ORCEnvironmentGetBodies(ptr, ptrs);
            DisposableList<Body> l = new DisposableList<Body>();
            foreach(IntPtr p in ptrs)
                l.Add(new Body(p));
            return l;
        }
    }

    public int RobotCount
    {
        get
        {
            return ORCEnvironmentGetRobots(ptr, null);
        }
    }

    public DisposableList<Robot> Robots
    {
        get
        {
            IntPtr[] ptrs = new IntPtr[RobotCount];
            ORCEnvironmentGetBodies(ptr, ptrs);
            DisposableList<Robot> l = new DisposableList<Robot>();
            foreach(IntPtr p in ptrs)
                l.Add(new Robot(p));
            return l;
        }
    }

    public void Add(InterfaceBase item)
    {
        ORCEnvironmentAdd(ptr, item.Ptr);
    }

    public void Remove(InterfaceBase item)
    {
        ORCEnvironmentRemove(ptr, item.Ptr);
    }

    public int AddModule(Module module, string args)
    {
        return ORCEnvironmentAddModule(ptr, module.Ptr, args);
    }

    public Module CreateModule(string name)
    {
        return new Module(ORCModuleCreate(ptr, name));
    }

    public ulong SimulationTime
    {
        get
        {
            return ORCEnvironmentGetSimulationTime(ptr);
        }
    }

    public void Lock()
    {
        ORCEnvironmentLock(ptr);
    }

    public void Unlock()
    {
        ORCEnvironmentUnlock(ptr);
    }

    public bool SetViewer(string viewername)
    {
        if(ORCEnvironmentSetViewer(ptr, viewername)==1)
            return true;
        else
            return false;
    }
}
}


using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace OpenRAVE
{
/// <summary>
/// The DebugLevel enum provides a means of specifying the level to which
/// debug information should be reported by OpenRAVE.
/// </summary>
public enum DebugLevel : uint {
    Fatal       = 0,
    Error       = 1,
    Warn        = 2,
    Info        = 3,
    Debug       = 4,
    Verbose     = 5,
    OutputMask  = 0xf,
    VerifyPlans = 0x80000000, ///< if set, should verify every plan returned. the verification is left up to the planners or the modules calling the planners. See \ref planningutils::ValidateTrajectory
};

/// <summary>
/// The Context class provides the main point of initialisation of the OpenRAVE
/// environment using a Singleton design pattern to restrict the a single
/// initialisation. I believe this is correct?
/// </summary>
public class Context : Base
{
    [DllImport("libopenrave0.9_c")]
    private static extern void ORCSetDebugLevel(uint level);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCInitialize(uint bLoadAllPlugins, uint level);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCDestroy();

    private static Context thisContext = null;
    private DebugLevel debugLevel;
    public DebugLevel DebugLevel
    {
        get
        {
            return debugLevel;
        }
        set
        {
            debugLevel = value;
            ORCSetDebugLevel((uint)value);
        }
    }

    private Context(DebugLevel level) : base((IntPtr)1)
    {
        debugLevel = level;
        ORCInitialize( 1, (uint)level);
    }

    public static Context Create()
    {
        return Create(DebugLevel.Info);
    }

    public static Context Create(DebugLevel level)
    {
        if(thisContext == null) thisContext = new Context(level);
        return thisContext;
    }

    protected override void Destroy()
    {
        ORCDestroy();
        thisContext = null;
    }

    public Environment CreateEnvironment()
    {
        return new Environment();
    }
}
}


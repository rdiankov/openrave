using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace OpenRAVE
{
/// <summary>
/// The BodyGeometry class provides access to geometry data of the
/// body.
/// </summary>
public class BodyGeometry : Base
{
    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern void ORCBodyLinkRelease(IntPtr link);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern void ORCBodyGeometrySetDiffuseColor(IntPtr geometry, float red, float green, float blue);

    internal BodyGeometry(IntPtr ptr) : base(ptr) {
    }

    protected override void Destroy()
    {
        // This is implied by the orcbodyfunctions.cpp example so we'll
        // maintain this assumption here.
        ORCBodyLinkRelease(ptr);
    }

    public void SetDiffuseColor(float red, float green, float blue)
    {
        ORCBodyGeometrySetDiffuseColor(ptr, red, green, blue);
    }
}

/// <summary>
/// The BodyLink class provides access to the link information which
/// joins elements the various geometrys of the model.
/// </summary>
public class BodyLink : Base
{
    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern void ORCBodyLinkRelease(IntPtr link);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern int ORCBodyLinkGetGeometries(IntPtr link, IntPtr[] geometries);

    internal BodyLink(IntPtr ptr) : base(ptr) {
    }

    protected override void Destroy()
    {
        ORCBodyLinkRelease(ptr);
    }

    public int GeometryCount
    {
        get
        {
            return ORCBodyLinkGetGeometries(ptr, null);
        }
    }

    public DisposableList<BodyGeometry> Geometries
    {
        get
        {
            IntPtr[] v = new IntPtr[GeometryCount];
            ORCBodyLinkGetGeometries(ptr, v);
            DisposableList<BodyGeometry> l = new DisposableList<BodyGeometry>();
            foreach(IntPtr p in v)
                l.Add(new BodyGeometry(p));
            return l;
        }
    }
}

/// <summary>
/// The Body class provides a broad range of support for the overall body
/// data of a model.
/// </summary>
public class Body : InterfaceBase
{
    [DllImport("libopenrave0.9_c")]
    private static extern IntPtr ORCBodyGetName(IntPtr body);

    [DllImport("libopenrave0.9_c", CharSet = CharSet.Ansi)]
    private static extern void ORCBodySetName(IntPtr body, string name);

    [DllImport("libopenrave0.9_c")]
    private static extern int ORCBodyGetDOF(IntPtr body);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodyGetDOFValues(IntPtr body, float[] values);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodyGetDOFValues(IntPtr body, double[] values);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodySetDOFValues(IntPtr body, float[] values);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodySetDOFValues(IntPtr body, double[] values);

    [DllImport("libopenrave0.9_c")]
    private static extern int ORCBodyGetLinks(IntPtr body, IntPtr[] links);

    /// \param[in] pose 7 values of the quaternion (4) and translation (3) of the world pose of the transform.
    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodySetTransform(IntPtr body, float[] pose);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodySetTransform(IntPtr body, double[] pose);

    /// \param[in] matrix column-order, row-major 3x4 matrix of the body world transform
    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodySetTransformMatrix(IntPtr body, float[] matrix);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodySetTransformMatrix(IntPtr body, double[] matrix);

    /// \param[out] pose 7 values of the quaternion (4) and translation (3) of the world pose of the transform.
    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodyGetTransform(IntPtr body, float[] pose);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodyGetTransform(IntPtr body, double[] pose);

    /// \param[out] matrix column-order, row-major 3x4 matrix of the body world transform
    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodyGetTransformMatrix(IntPtr body, float[] matrix);

    [DllImport("libopenrave0.9_c")]
    private static extern void ORCBodyGetTransformMatrix(IntPtr body, double[] matrix);

    [DllImport("libopenrave0.9_c")]
    private static extern int ORCBodyInitFromTrimesh(IntPtr body, IntPtr trimesh, uint visible);

    internal Body(IntPtr ptr) : base(ptr) {
    }

    public string Name
    {
        get
        {
            return Marshal.PtrToStringAnsi(ORCBodyGetName(ptr));
        }
        set
        {
            ORCBodySetName(ptr, value);
        }
    }

    public int DOF
    {
        get
        {
            return ORCBodyGetDOF(ptr);
        }
    }

    public float[] DOFValuesF
    {
        get
        {
            float[] v = new float[DOF];
            ORCBodyGetDOFValues(ptr, v);
            return v;
        }
        set
        {
            ORCBodySetDOFValues(ptr, value);
        }
    }

    public double[] DOFValuesD
    {
        get
        {
            double[] v = new double[DOF];
            ORCBodyGetDOFValues(ptr, v);
            return v;
        }
        set
        {
            ORCBodySetDOFValues(ptr, value);
        }
    }

    public int LinkCount
    {
        get
        {
            return ORCBodyGetLinks(ptr, null);
        }
    }

    public DisposableList<BodyLink> Links
    {
        get
        {
            IntPtr[] v = new IntPtr[LinkCount];
            ORCBodyGetLinks(ptr, v);
            DisposableList<BodyLink> l = new DisposableList<BodyLink>();
            foreach(IntPtr p in v)
                l.Add(new BodyLink(p));
            return l;
        }
    }

    public float[] TransformF
    {
        get
        {
            float[] f = new float[7];
            ORCBodyGetTransform(ptr, f);
            return f;
        }
        set
        {
            if(value.Length != 7)
                return;
            ORCBodySetTransform(ptr, value);
        }
    }

    public double[] TransformD
    {
        get
        {
            double[] f = new double[7];
            ORCBodyGetTransform(ptr, f);
            return f;
        }
        set
        {
            if(value.Length != 7)
                return;
            ORCBodySetTransform(ptr, value);
        }
    }

    public float[] TransformMatrixF
    {
        get
        {
            float[] f = new float[12];
            ORCBodyGetTransformMatrix(ptr, f);
            return f;
        }
        set
        {
            if(value.Length != 12)
                return;
            ORCBodySetTransformMatrix(ptr, value);
        }
    }

    public double[] TransformMatrixD
    {
        get
        {
            double[] f = new double[12];
            ORCBodyGetTransformMatrix(ptr, f);
            return f;
        }
        set
        {
            if(value.Length != 12)
                return;
            ORCBodySetTransformMatrix(ptr, value);
        }
    }

    public bool InitFromTriMesh(TriMesh trimesh, bool visible)
    {
        if (visible)
            return ORCBodyInitFromTrimesh(ptr, trimesh.Ptr, 1);
        else
            return ORCBodyInitFromTrimesh(ptr, trimesh.Ptr, 0);
    }

    protected override void Destroy()
    {
        // How??
    }
}

}


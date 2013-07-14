using OpenRAVE;

namespace customgeometry
{
	class MainClass
	{
		public static void Main (string[] args)
		{
			using(Context c = Context.Create(DebugLevel.Debug))
			{
				using(Environment e = c.CreateEnvironment())
				{
					float[] vertices = new float[] { 1,0,0, 0,1,0,  0,0,1 };
					float[] indices = new float[] { 0,1,2 };
					
					using(TriMesh trimesh = new TriMesh(vertices, indices))
					{
						using(Body body = e.CreateBody(""))
						{
							body.InitFromTriMesh(trimesh, true);
							body.Name = "mytriangle";
							
							e.Add (body);
							e.SetViewer("qtcoin");
							
							float[] pose = new float[] { 1,0,0,0,0,0,0 };
							ulong startsimtime = e.SimulationTime;
							
							for(int i = 0; i < 10000; ++i) {
								e.Lock();
								
								ulong newsimtime = e.SimulationTime;
								double deltatime = (double)((newsimtime-startsimtime)*1e-6);
								//double fanim = System.Math.IEEERemainder(deltatime,1.0);
								
								pose[0] = (float)System.Math.Sin(deltatime);
								pose[3] = (float)System.Math.Cos(deltatime);
								body.TransformF = pose;
								
								e.Unlock();

								// wait until sim time changes
						        while(newsimtime == e.SimulationTime)
						            System.Threading.Thread.Sleep(1);
							}
						}
					}
				}
			}
		}
	}
}

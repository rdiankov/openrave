using OpenRAVE;

namespace bodyfunctions
{
	class MainClass
	{
		public static void Main (string[] args)
		{
			using(Context c = Context.Create(DebugLevel.Debug))
			{
				using(Environment e = c.CreateEnvironment())
				{
					e.Load("data/lab1.env.xml");
					e.SetViewer("qtcoin");
					
					bool pole2added = true;
					Body bodypole2 = e.GetBody("pole2");
					Body bodypole3 = e.GetBody("pole3");
					
					using(DisposableList<BodyLink> links = bodypole3.Links)
					{
						using(DisposableList<BodyGeometry> geometries = links[0].Geometries)
						{
					
							float[] pose = bodypole3.TransformF;
							ulong startsimtime = e.SimulationTime;;
							
							for(int i = 0; i < 10000; ++i) {
						        e.Lock();				
						        
								ulong newsimtime = e.SimulationTime;
								double deltatime = (newsimtime-startsimtime)*1e-6;
						        float fanim = (float)System.Math.IEEERemainder(deltatime,1.0);
						        
								// animate around X axis
						        pose[4] = 1.0f + fanim;
								bodypole3.TransformF = pose;
						
						        // set the color
								geometries[0].SetDiffuseColor(1.0f, 0.0f, fanim);

						        if(fanim > 0.5 && pole2added)
								{
						            // remove the pole
									e.Remove(bodypole2);
						            pole2added = false;
						        }
						        else if(fanim < 0.5 && !pole2added)
								{
									e.Add(bodypole2);
						            pole2added = true;
						        }
						
						        // unlock
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

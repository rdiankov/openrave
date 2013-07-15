using OpenRAVE;

namespace rrtplanning
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
					
					using(DisposableList<Robot> robots = e.Robots)
					{
						string robotname = robots[0].Name;
						System.Console.WriteLine("robot name is: " + robotname);
						
						using(Module basemanip = e.CreateModule("BaseManipulation"))
						{
							e.AddModule(basemanip, robotname);
							string output = basemanip.SendCommand("MoveManipulator goal -0.75 1.24 -0.064 2.33 -1.16 -1.548 1.19 outputtraj execute 0");
							System.Console.WriteLine("rrt output is: " + output);					
						}					
					}
				}
			}
		}
	}
}

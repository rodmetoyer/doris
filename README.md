# [doris](https://en.wikipedia.org/wiki/Doris_(mythology))
Modeling and simulation software for a dual-rotor tethered fluid kinetic-energy extraction device.
More documentation is on the way. For now, if you are not working directly with the author then you may have trouble running this code.

## Want to run something
The DualRotorSim.m script is how you run simulation scenarios. Open that file in Matlab and click run and you should see something happen. However, you may see something happen for hours and hours. If you are not working directly with the author, you may want to spend some time familiarizing yourself with the classes before you start running code.

### Input Files
The code runs off of input files. Generally, it is on you to create the input file. If you look in the tools folder you'll find an input maker tool. There is one input file premade in the repo (one I know for sure is working anyway): BLBcase1.m. That is the default input file in DualRotorSim.m. You can use it as a template or use the tool in the tools folder to create new input files.

### Simulation class
When you run DualRotorSim you will end up with an object of class simulation called sim. You can do a lot with that guy. Check out the available methods in the simulation class. that object has all of the simulation objects as properties. You get to their methods through the sim object. For example, type sim.vhcl.visualizeSectionLoads and see what pops up.

## Info (In case you have stumbled on this by accident)
This code is still in development. It is not currently meant for public consumption. There are probably bugs. And there is no documentation. And it is a hacky mess. But it works for our current modeling objectives, we're getting good results, and we are working on publishing those results.

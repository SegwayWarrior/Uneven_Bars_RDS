# Team Uneven Bars
## Robot Design Studio 2020 Northwestern University


Hello! This is the group repo for the Uneven Bars team of RDS 2020.  We ended up sharing much of our code with each other, so you'll see some redundant examples throughout the directories.  Here is a list of how to run specific examples that were shown throughout our presentation. [Demo-Day Presentation](https://docs.google.com/presentation/d/1LsOqx8bDn7tv1ctSCxElXI1K3S5CPRHfwvgiGdfVRC0/edit?usp=sharing)

![Clear-Hip Hecht](Energy_Release_Catch/release_catch.gif)
This is a simulation of the Clear-Hip Hecht, or going from low to high bar.  In order to run the code, open [Energy_Release_Catch](https://github.com/SegwayWarrior/Uneven_Bars_RDS/tree/master/Energy_Release_Catch) and run main_uneven_release.m in Matlab.  

![Energy-Pumping](Energy_Pumping/energy_pumping.gif) This is the energy controller starting from rest.  The energy controller is based on a [paper](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.467.5126&rep=rep1&type=pdf) written in 1996 by Mark W. Spong of University of Illinois.  In order to run the code, open [Energy_Pumping](https://github.com/SegwayWarrior/Uneven_Bars_RDS/tree/master/Energy_Pumping) and run main_uneven_pumping.m in Matlab.  

![Successful Trajectories](select_parameters/example1.jpg)
To see how we selected key state parameters for a successful release-to-catch, go to folder [select_parameters](https://github.com/SegwayWarrior/Uneven_Bars_RDS/tree/master/select_parameters).

To see a single phase of releasing the bar, go to folder [Release_bar](https://github.com/SegwayWarrior/Uneven_Bars_RDS/tree/master/Release_Bar) and main_releasebar.m, derive_equations_releasebar.m is the file about constructing model. 

To see double phase of releasing and catching bar, go to folder [Release&&Catch](https://github.com/SegwayWarrior/Uneven_Bars_RDS/tree/master/Release%26%26Catch). 

Live script is here [live script](https://github.com/SegwayWarrior/Uneven_Bars_RDS/blob/master/Live%20Script/Triple_Pendulum_Live_Script.mlx)

## Future Plans
Moving forward, we plan on updating our simulation with the actual robot parameters, and produce a comparison of the required torques for each controller. (Energy, PID, and Torque)  We also plan on simulating the Tkatchev as well as balancing above the bar.

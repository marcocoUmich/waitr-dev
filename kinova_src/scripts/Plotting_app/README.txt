The main changes between this fork and base waitr are:
1. uarmtd_agent and uarmtd_simulator store a lot of constraints/braking maneuvers/etc
2. uarmtd_planner is equipped with more parameters (s_thresh, n_timesteps, Duration, k_range). These can be passed to the 
cuda program without needing to change in both places. All of them can just be changed here, except n_timesteps requires special
functionality to do so (see the function at the bottom of plot_100_planners). It makes a header and writes the param to that, then recompiles.
3. Lots of plotting stuff. This folder contains my plotting things, described below:












This folder contains plotting functionality for armour/waitr. The main function is plotting.m

To run plotting.m, if you want to plot an experiment, like a k_range sweep or something, run that experiment in:
./scripts/kinova_run_100_planners.m

Then, change the filepath at the top of plotting.m to be the experiment output folder (should end with KR, ST, or TS).
Running the app should then let you do comparison plots.



The more basic plotting functionality requires an Agent to be loaded in the workspace. The help menu says what each type of plot
requires. (Some need things like braking maneuvers saved, which my uarmtd_agent does by default).




Right now, force reach set plotting doesn't work. I save a bunch of force stuff in uarmtd_agent.m and uarmtd_simulator.m, and 
started a function plot_force.m for plotting them, but they don't display. This needs some debugging if it is wanted.


In plotting_app folder there is also an example k_range sweep experiment.
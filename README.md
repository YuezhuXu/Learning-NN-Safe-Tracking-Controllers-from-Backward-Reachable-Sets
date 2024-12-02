# Learning-NN-Safe-Tracking-Controllers-from-Backward-Reachable-Sets
We obtain nominal trajectories using Planning_Dubin_Car.m and then generate backward reachable sets using Tracking_Dubin_Car.m. The visualization of the backward reachable sets can be done through Plotting_Dubin_Car.m. We generate sampling points for each time step by TWoBRS_Dubin_Car. With data, we train our NN safe tracking controllers using training_controller_multiplyarch.py. We conduct simulations to test our controllers and plot the results using Testing_NNcontroller.m.

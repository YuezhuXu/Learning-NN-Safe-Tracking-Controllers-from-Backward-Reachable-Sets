# Learning-NN-Safe-Tracking-Controllers-from-Backward-Reachable-Sets
## Data Preparation
We obtain nominal trajectories using `Planning_Dubin_Car.m` and then generate backward reachable sets using `Tracking_Dubin_Car.m`. The visualization of the backward reachable sets can be done through `Plotting_Dubin_Car.m`. We generate sampling points for each time step by `TWoBRS_Dubin_Car.m`. 

## Training
With data, we train our NN safe tracking controllers using `learning_BRS_instructed/training_controller_multiplyarch.py`. 

For benchmarks, NN controllers trained without BRS are obtained using `learning_without_BRS/training_controller_box_multiplyarch.py`. 

For the CBF-based controller, the control input for each step can be solved through 
`solving_with-CBF/solve_cbf_dubins_one_step.m` (directly implemented in `Testing_NNcontroller.m`).

## Testing
We conduct simulations to test our controllers and plot the results using `Testing_NNcontroller.m`.

# -*- coding: utf-8 -*-
"""
Created on Fri Oct 18 17:41:09 2024

@author: 22384
"""


# We normalize states x to x-c1 (or x-xc) before it is input in the trainable feedforward part of NN 
# Biases are kept (x-xc)*NN_{before sigmoid}

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
#import matplotlib.pyplot as plt
import numpy as np
import scipy.io
import os
from sklearn.model_selection import train_test_split  # For splitting the data
import time
import random


seed = 42
random.seed(seed)
np.random.seed(seed)  # NumPy
torch.manual_seed(seed)  # PyTorch (CPU)
torch.cuda.manual_seed(seed)  # PyTorch (GPU)
torch.cuda.manual_seed_all(seed)  # If using multiple GPUs



device = torch.device("cuda" if torch.cuda.is_available() else "cpu")



def F_Dubin_Car(x, u):
    
    cos_theta = torch.cos(x[:, 2])  # Apply element-wise cos
    sin_theta = torch.sin(x[:, 2])  # Apply element-wise sin
    
    transformation_matrix = torch.stack([torch.stack([cos_theta, torch.zeros_like(cos_theta)], dim=1),
                                         torch.stack([sin_theta, torch.zeros_like(sin_theta)], dim=1),
                                         torch.stack([torch.zeros_like(cos_theta), torch.ones_like(cos_theta)], dim=1)], dim=1)
    
    y = x + torch.bmm(transformation_matrix, u.unsqueeze(2)).squeeze(2)
    
    return y

def init_he(m):
    if isinstance(m, nn.Linear):
        nn.init.kaiming_uniform_(m.weight, nonlinearity='relu')
        # nn.init.zeros_(m.weight)
        
        
def save_model_params(model, filename):
    params = {}
    for name, param in model.named_parameters():
        # Replace periods in parameter names with underscores for MATLAB compatibility
        safe_name = name.replace('.', '_')
        # Convert each parameter tensor to a NumPy array
        param_np = param.detach().cpu().numpy().astype(np.float64)  # Ensure compatible data type
        params[safe_name] = param_np
    if 'Ru' not in params:
        Ru_trained = model.Ru.detach().cpu().numpy().astype(np.float64)
        params['Ru'] = Ru_trained
        
    # Save in MATLAB v5 .mat format
    scipy.io.savemat(filename, params)


    
# Get data from saved .mat file
#rl: 55,90; lu: 66,102; ll: 77,105
sd = 77 #55,66,77

if sd == 55:
    ini_str = 'rl'
    numNodes = 90
elif sd == 66:
    ini_str = 'lu'
    numNodes = 77
elif sd == 77:
    ini_str = 'll'
    numNodes = 68
    
    
    

start_all = time.time()
for ind in range(1,numNodes+1):
    start = time.time()

    data_ex = scipy.io.loadmat('sampling/states_sampled_extreme_z'+str(ind)+'_rand'+str(sd)+'_'+ini_str+'.mat')
    data_uni = scipy.io.loadmat('sampling/states_sampled_uniform_z'+str(ind)+'_rand'+str(sd)+'_'+ini_str+'.mat')
    
    # Same in all
    cu = torch.tensor(data_uni['cu'], dtype=torch.float32).flatten().to(device)
    Ru = torch.tensor(data_uni['Ru'], dtype=torch.float32).flatten().to(device)
    c1 = torch.tensor(data_uni['c1'].flatten(), dtype=torch.float32).to(device)
    c2 = torch.tensor(data_uni['c2'].flatten(), dtype=torch.float32).to(device)
    G2 = data_uni['G2']
    G_pseudoinverse = torch.tensor(np.linalg.pinv(G2), dtype=torch.float32).to(device)
    
    
    
    states_ex = data_ex['states_ex'].T  # Shape [x, 3]
    states_uni = data_uni['states_uni'].T 
    states_uniex = np.concatenate((states_uni, states_ex), axis=0)  
    
    
    # Split data into training (70%) and test (30%) sets from the uniform set
    states_train, states_val = train_test_split(states_uniex, test_size=0.3)
    
    
    # Define the Neural Network
    class CustomNN(nn.Module):
        def __init__(self, input_size, hidden_size, output_size, c1, cu, initial_Ru):
            super(CustomNN, self).__init__()
            
            # Adding biases for each layer
            self.fc1 = nn.Linear(input_size, hidden_size, bias=True)  
            self.fc2 = nn.Linear(hidden_size, hidden_size, bias=True)
            self.fc3 = nn.Linear(hidden_size, hidden_size, bias=True)
            self.fc4 = nn.Linear(hidden_size, hidden_size, bias=True) # too deep does not help
            self.fclinkback =  nn.Linear(hidden_size, input_size, bias=True)
            self.fcout = nn.Linear(input_size, output_size, bias=False)  
            

            
            # Store cu and Ru for transformation
            self.cu = cu
            # Initialize Ru as a trainable parameter
            self.Ru = nn.Parameter(torch.zeros_like(initial_Ru))#torch.tensor(initial_Ru, dtype=torch.float32))  # initial_Ru is a 2x1 vector
            self.c1 = c1
            


        def forward(self, x):
            x_normalized = x - self.c1  # Normalize with respect to c1
    
            # Pass through the trainable layers
            x = torch.relu(self.fc1(x_normalized))  # First hidden layer
            x = torch.relu(self.fc2(x))  
            x = torch.relu(self.fc3(x))  
            x = torch.relu(self.fc4(x))  
            x = torch.relu(self.fclinkback(x))  
            
            
            # Multiply the input difference (x - c1) before applying sigmoid to the output
            x = x * x_normalized  # Multiply by (x - c1)
            
            x= self.fcout(x) #torch.relu(self.fcout(x))
            
            
            
            # Tanh to scale the output to the range [-1, 1]
            x = torch.tanh(x)
            
            
            # Transform output to the range [cu - Ru, cu + Ru]
            u = self.cu + x * self.Ru  # Scale and shift to [cu - Ru, cu + Ru]

            
            return u
    
    
    # Create the neural network instance
    input_size = 3  # Example input dimension
    hidden_size = 32
    output_size = 2
    
    c1 = c1.clone().detach().requires_grad_(True) #torch.tensor(c1, dtype=torch.float32)
    c2 = c2.clone().detach().requires_grad_(True) #torch.tensor(c2, dtype=torch.float32)
    net = CustomNN(input_size, hidden_size, output_size, c1, cu, Ru).to(device)
    net.apply(init_he)
    
    # Optimizer
    optimizer = optim.Adam(net.parameters(), lr=1e-3, weight_decay=1e-5)
    

        
    # Define the mean squared error loss
    loss_fn = nn.MSELoss()

    
    num_batches = 1 
    batch_size = len(states_train) // num_batches
    
    
    
    prev_loss = 1e10
    patience = 5  # Number of epochs to wait for an improvement
    patience_counter = 0  # Counter to keep track of epochs without improvement
    best_val_loss = float('inf')  # Initialize best validation loss with infinity
    for epoch in range(10000):  # Example number of epoc
        
        for i in range(num_batches):
            # Get batch of data based on the current train_indices
            start_idx = i * batch_size
            end_idx = (i + 1) * batch_size
    
            x_batch = torch.tensor(states_train[start_idx:end_idx, :], dtype=torch.float32).to(device)
           
            
            # Forward pass
            u_batch = net(x_batch)
            y_batch = F_Dubin_Car(x_batch, u_batch)
    
            # Expand c2 to match y_batch's shape
            c2_expanded = c2.repeat(y_batch.shape[0], 1)  # Repeat along the batch size
    
            # Compute the residual f(x^s_k, Π(x^s_k)) - c2
            residual = y_batch - c2_expanded
    
            # Apply the pseudoinverse inside the norm
            residual_pinv = torch.matmul(G_pseudoinverse, residual.T).T  
            
            infinity_norms = torch.norm(residual_pinv, p=float('inf'), dim=1)
            
            # Compute the squared norm for each residual, multiplied by the batch-specific coefficients
            loss = 100 * torch.mean(torch.exp(torch.relu(infinity_norms - 1)) + torch.norm(residual_pinv, dim=1, p = float('inf'))) + torch.norm(net.Ru, p=2)


 
            # Backpropagation and optimization step
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        
        
        
        # Check validation loss every 100 epochs
        if epoch % 100 == 0:
            net.eval()
            with torch.no_grad():
                x_val = torch.tensor(states_val, dtype=torch.float32).to(device)
                u_val = net(x_val)
                y_val = F_Dubin_Car(x_val, u_val)
                residual_val = y_val - c2.repeat(y_val.shape[0], 1)
                residual_pinv_val = torch.matmul(G_pseudoinverse, residual_val.T).T
                val_loss_mean = torch.mean(torch.norm(residual_pinv_val, dim=1, p=float('inf')))
                print(f"Validation loss at epoch {epoch}: ", val_loss_mean.item())
            
            # Early stopping check
            if val_loss_mean < best_val_loss:
                best_val_loss = val_loss_mean  # Update best validation loss
                patience_counter = 0  # Reset patience counter
                # Save the best model parameters
                best_params = {name: param.clone().detach() for name, param in net.named_parameters()}
            else:
                patience_counter += 1  # Increment patience counter if no improvement
                
                # Stop training if patience has been exceeded
                if patience_counter >= patience:
                    break
     
            net.train()  # Switch back to training mode after validation
        
        
    # Create the target directory if it doesn't exist
    output_dir = 'trained_controller_each_step'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
        
    if best_params is not None:
        for name, param in net.named_parameters():
            param.data.copy_(best_params[name])  # Load best parameters
            
        # To make it compatible to matlab indices, we store the index of the neural controllers starting from 1 (i.e. for time step 0, it is step1 here)
        save_model_params(net, f"trained_controller_each_step/model_params_step{ind}_rand{sd}_"+ini_str+".mat")
        print(f"Model parameters saved.")

   
    
    
    finish = time.time()
    print(f"Time used for step {ind} is", finish-start)



finish_all = time.time()
print(f"Total time used is", finish_all-start_all)


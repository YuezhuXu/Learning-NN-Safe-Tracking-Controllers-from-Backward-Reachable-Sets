# We normalize states x to x-c1 (or x-xc) before it is input in the trainable feedforward part of NN 
# Biases are kept (x-xc)*NN_{before sigmoid}

from tqdm import tqdm
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import scipy.io
import os
from sklearn.model_selection import train_test_split  # For splitting the data
import time
import random
from torch.utils.data import DataLoader, TensorDataset
from multiprocessing import Pool
import datetime




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
        self.Ru = nn.Parameter(torch.zeros_like(initial_Ru))#torch.tensor(initial_Ru, dtype=torch.float32))  # initial_Ru is a 2x1 vector
        self.c1 = c1
        
        """
        # Clamping bounds for Ru within the network
        self.Ru_min = torch.tensor([max(self.cu[0] - 0.5, -0.5 - self.cu[0]), max(self.cu[1] - 0.3, -0.3 - self.cu[1])]).to(self.cu.device)
        self.Ru_max = torch.tensor([min(0.5 - self.cu[0], self.cu[0] + 0.5), min(0.3 - self.cu[1], self.cu[1] + 0.3)]).to(self.cu.device)
        """
        

    def forward(self, x):
        # Normalize the input before passing through the first layer
        x_normalized = x - self.c1  # Normalize with respect to c1

        # Pass through the trainable layers
        x = torch.relu(self.fc1(x_normalized))  # First hidden layer
        x = torch.relu(self.fc2(x))  
        x = torch.relu(self.fc3(x))  
        x = torch.relu(self.fc4(x))  
        x = torch.relu(self.fclinkback(x))  
        
        
        # Multiply the input difference (x - c1) before applying sigmoid to the output
        x = x * x_normalized  # Multiply by (x - c1)
        
        x= self.fcout(x) 
        
        # Tanh to scale the output to the range [-1, 1]
        x = torch.tanh(x)
        
        
        
        # Transform output to the range [cu - Ru, cu + Ru]
        u = self.cu + x * self.Ru  # Scale and shift to [cu - Ru, cu + Ru]
        
        
        return u

def train_controller_for_step(ind, sd, ini_str, numNodes, device):
    start = time.time()
    log_dir = "training_logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"log_step{ind}_rand{sd}_{ini_str}.txt")
    
    with open(log_file, "w", buffering=1) as f:  # line-buffered, flushes per line
        f.write(f"[{datetime.datetime.now()}] Starting step {ind}\n")
        
    
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
    
        states_train_tensor = torch.tensor(states_train, dtype=torch.float32)  # [N, 3]
        states_val_tensor   = torch.tensor(states_val, dtype=torch.float32)    # [M, 3]
        
        train_dataset = TensorDataset(states_train_tensor)
        train_loader = DataLoader(train_dataset, batch_size=256, shuffle=True)  # Tune batch size
    
        
        
       
        
        # Create the neural network instance
        input_size = 3  # Example input dimension
        hidden_size = 32
        output_size = 2
        
        
        c2_val = c2.repeat(states_val_tensor.shape[0], 1) 
        
        
        net = CustomNN(input_size, hidden_size, output_size, c1, cu, Ru).to(device)
        net.apply(init_he)
        
        # Optimizer
       
        optimizer = optim.Adam(net.parameters(), lr=3e-4, weight_decay=1e-4)
    
        
        # Training loop
        num_batches = 1
        batch_size = len(states_train) // num_batches
    
    
        #previous_loss_value = float('inf')  # Track the previous loss (start with a high value)
        prev_loss = float('inf')
        patience = 5  # Number of epochs to wait for an improvement
        patience_counter = 0  # Counter to keep track of epochs without improvement
        best_val_loss = float('inf')  # Initialize best validation loss with infinity
        for epoch in range(10000):  # Example number of epoc
            
            for x_batch_tuple in train_loader:
                
                x_batch = x_batch_tuple[0].to(device)
               
                
                # Forward pass
                u_batch = net(x_batch)
                y_batch = F_Dubin_Car(x_batch, u_batch)
                
                '''
                # Expand c2 to match y_batch's shape
                c2_expanded = c2.repeat(y_batch.shape[0], 1)  # Repeat along the batch size
                '''
                
                # Compute the residual f(x^s_k, Π(x^s_k)) - c2
                residual = y_batch - c2
        
                # Apply the pseudoinverse inside the norm
                residual_pinv = torch.matmul(G_pseudoinverse, residual.T).T  # G_pseudoinverse * residual
                # G2 helps

                
                infinity_norms = torch.norm(residual_pinv, p=float('inf'), dim=1)
                
                 loss = torch.mean(torch.exp(torch.relu(infinity_norms - 1))) + 10 * torch.mean(infinity_norms) + 0.01 * torch.norm(net.Ru, p=1)
    
                
                
    
     
                # Backpropagation and optimization step
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
            
            
            
            # Check validation loss every 100 epochs
            if epoch % 200 == 0:
                net.eval()
                with torch.no_grad():
                    x_val = torch.tensor(states_val, dtype=torch.float32).to(device)
                    u_val = net(x_val)
                    y_val = F_Dubin_Car(x_val, u_val)
                    residual_val = y_val - c2_val #c2.repeat(y_val.shape[0], 1)
                    residual_pinv_val = torch.matmul(G_pseudoinverse, residual_val.T).T
                    val_loss_mean = torch.mean(torch.norm(residual_pinv_val, dim=1, p=float('inf')))
                    # print(f"Validation loss at epoch {epoch}: ", val_loss_mean.item())
                
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
        
            
        if best_params is not None:
            for name, param in net.named_parameters():
                param.data.copy_(best_params[name])  # Load best parameters
                
            # To make it compatible to matlab indices, we store the index of the neural controllers starting from 1 (i.e. for time step 0, it is step1 here)
            save_model_params(net, f"trained_controller_each_step/model_params_step{ind}_rand{sd}_"+ini_str+".mat")
            print(f"Model {ind} parameters saved.")
    
        duration = datetime.timedelta(seconds=int(time.time() - start))
        f.write(f"[{datetime.datetime.now()}] Finished step {ind} in {duration}\n")
    
       
    
if __name__ == "__main__":
    seed = 123
    random.seed(seed)
    np.random.seed(seed)  # NumPy
    torch.manual_seed(seed)  # PyTorch (CPU)
    torch.cuda.manual_seed(seed)  # PyTorch (GPU)
    torch.cuda.manual_seed_all(seed)  # If using multiple GPUs



    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        
    # Get data from saved .mat file
    #rl: 333, 86; lu: 222, 73; ll: 111, 67
    sd = 111

    if sd == 333:
        ini_str = 'rl'
        numNodes = 86
    elif sd == 222:
        ini_str = 'lu'
        numNodes = 73
    elif sd == 111:
        ini_str = 'll'
        numNodes = 67
        
        
    # Create the target directory if it doesn't exist
    output_dir = 'trained_controller_each_step'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        

    
    # Starting parallel training  
    # 12 for local, 64 for cluster
    num_workers = min(12, numNodes)
    start_all = time.time()
    start_all_dt = datetime.datetime.now()
    
    tasks = [(i, sd, ini_str, numNodes, device) for i in range(1, numNodes + 1)]    #range(1, numNodes + 1)
    
    with Pool(processes=num_workers) as pool:
        pool.starmap(train_controller_for_step, tasks)
    
    finish_all = time.time()
    finish_all_dt = datetime.datetime.now()
    total_duration = datetime.timedelta(seconds=int(finish_all - start_all))
    
    print(f"[{finish_all_dt}] All {numNodes} controllers completed.")
    print(f"[SUMMARY] Total training time: {total_duration}")

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



def sample_uniform_box(center, half_lengths, n_samples):
    """
    Uniform sampling in an axis-aligned box:
        [center - half_lengths, center + half_lengths]
    center, half_lengths: shape [d]
    returns: [n_samples, d]
    """
    low = center - half_lengths
    high = center + half_lengths
    return np.random.uniform(low=low, high=high, size=(n_samples, center.shape[0]))


def train_controller_for_step_box(ind, sd, ini_str, numNodes, device,
                                  X_safe, Rx_safe, n_samples=100):
    """
    ind is 1-based, matching MATLAB-style indexing.

    Assumption:
      - X_safe[:, k] is the safe-box center at step k+1 in Python 0-based indexing.
      - For controller step ind, we sample x_k from box ind,
        and use X_safe[:, ind] as the next-step reference x_{k+1}.
    """
    start = time.time()

    log_dir = "training_logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"log_step{ind}_rand{sd}_{ini_str}_box.txt")

    with open(log_file, "w", buffering=1) as f:
        f.write(f"[{datetime.datetime.now()}] Starting step {ind} with safe box sampling\n")

        # keep using the old per-step file for cu / Ru
        data_uni = scipy.io.loadmat(
            f"sampling/states_sampled_uniform_z{ind}_rand{sd}_{ini_str}.mat"
        )

        cu = torch.tensor(data_uni['cu'], dtype=torch.float32).flatten().to(device)
        Ru = torch.tensor(data_uni['Ru'], dtype=torch.float32).flatten().to(device)

        # current safe box center / half-length
        xk_center_np = X_safe[:, ind - 1].astype(np.float32)
        xk_half_np   = Rx_safe[:, ind - 1].astype(np.float32)

        # use current box center for normalization in the NN
        c1 = torch.tensor(xk_center_np, dtype=torch.float32).to(device)

        # target x_{k+1}: next box center
        xkp1_ref_np = X_safe[:, ind].astype(np.float32)
        xkp1_ref = torch.tensor(xkp1_ref_np, dtype=torch.float32).to(device)
        xkp1_ref_batch = xkp1_ref.unsqueeze(0)

        # sample x_k from the safe box
        states_all = sample_uniform_box(xk_center_np, xk_half_np, n_samples)

        states_train, states_val = train_test_split(
            states_all, test_size=0.3, random_state=42
        )

        states_train_tensor = torch.tensor(states_train, dtype=torch.float32)
        states_val_tensor = torch.tensor(states_val, dtype=torch.float32)

        num_batches = 1
        batch_size = max(1, len(states_train) // num_batches)

        train_dataset = TensorDataset(states_train_tensor)
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)

        input_size = 3
        hidden_size = 32
        output_size = 2

        net = CustomNN(input_size, hidden_size, output_size, c1, cu, Ru).to(device)
        net.apply(init_he)

        optimizer = optim.Adam(net.parameters(), lr=1e-2, weight_decay=1e-4)

        patience = 5
        patience_counter = 0
        best_val_loss = float('inf')
        best_params = None

        for epoch in range(10000):
            net.train()

            for x_batch_tuple in train_loader:
                x_batch = x_batch_tuple[0].to(device)

                u_batch = net(x_batch)
                y_batch = F_Dubin_Car(x_batch, u_batch)

                # NEW objective: mean infinity norm of f(x_k,u_k) - x_{k+1}
                residual = y_batch - xkp1_ref_batch
                p1 = torch.mean(torch.norm(residual, p=float('inf'), dim=1))
                p2 = torch.norm(net.Ru, p=1)

                loss = 10 * p1 + 0.01 * p2

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            if epoch % 200 == 0:
                net.eval()
                with torch.no_grad():
                    x_val = states_val_tensor.to(device)
                    u_val = net(x_val)
                    y_val = F_Dubin_Car(x_val, u_val)

                    residual_val = y_val - xkp1_ref_batch
                    val_loss_mean = torch.mean(torch.norm(residual_val, p=float('inf'), dim=1))

                print(f"Step {ind}, epoch {epoch}, val={val_loss_mean.item():.6e}")
                f.write(f"[{datetime.datetime.now()}] Epoch {epoch}, val={val_loss_mean.item():.6e}\n")

                if val_loss_mean < best_val_loss:
                    best_val_loss = val_loss_mean
                    patience_counter = 0
                    best_params = {
                        name: param.clone().detach()
                        for name, param in net.named_parameters()
                    }
                else:
                    patience_counter += 1
                    if patience_counter >= patience:
                        break

        if best_params is not None:
            for name, param in net.named_parameters():
                param.data.copy_(best_params[name])

            os.makedirs("trained_controller_each_step", exist_ok=True)
            save_model_params(
                net,
                f"trained_controller_each_step/model_params_step{ind}_rand{sd}_{ini_str}_box.mat"
            )
            print(f"Model {ind} parameters saved.")

        duration = datetime.timedelta(seconds=int(time.time() - start))
        f.write(f"[{datetime.datetime.now()}] Finished step {ind} in {duration}\n")


if __name__ == "__main__":
    seed = 123
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

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

    output_dir = 'trained_controller_each_step'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # load safe boxes from MATLAB file
    box_data = scipy.io.loadmat(f'Dubin_Car_Data_For_Plotting_{sd}_{ini_str}.mat')
    

    X_safe = np.asarray(box_data['X'], dtype=np.float32)
    Rx_safe = np.asarray(box_data['Rx'], dtype=np.float32)

    # in case MATLAB saved them as (#steps, 3)
    if X_safe.shape[0] != 3 and X_safe.shape[1] == 3:
        X_safe = X_safe.T
        Rx_safe = Rx_safe.T

    assert X_safe.shape == Rx_safe.shape, "X and Rx must have the same shape"
    assert X_safe.shape[0] == 3, "Safe boxes should be 3 x (#steps)"

    print('Start training with safe-box sampling')
    start_all = time.time()

    # need ind and ind+1, so stop at the second-to-last box
    max_step = min(numNodes, X_safe.shape[1] - 1)

    for i in range(1, max_step + 1):
        print(f"Step {i} with safe-box sampling")
        train_controller_for_step_box(
            i, sd, ini_str, numNodes, device,
            X_safe, Rx_safe,
            n_samples=1000
        )

    finish_all = time.time()
    total_duration = datetime.timedelta(seconds=int(finish_all - start_all))

    print(f"All {max_step} controllers completed.")
    print(f"[SUMMARY] Total training time: {total_duration}")
    

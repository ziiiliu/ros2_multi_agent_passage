import torch
import torch.optim as optim
import torch.nn as nn
import numpy as np

class ActiveNet(nn.Module):
    def __init__(self, input_dim, output_dim, n_visible=1, hidden_dim=64):
        super(ActiveNet, self).__init__()
        self.input = nn.Linear(input_dim, hidden_dim)
        self.output = nn.Linear(hidden_dim, output_dim)
        self.activation = nn.ReLU()

    def forward(self, x):
        x = self.activation(self.input(x))
        x = self.output(x)
        # mean = x[:,0][:, None]
        # std = torch.clamp(x[:,1][:, None], min=0.01)
        # normal_dist = torch.distributions.Normal(mean, std)
        return x

def gaussian_nll(y_pred, y_true):
    """
    Gaussian negative log likelihood
    
    Note: to make training more stable, we optimize
    a modified loss by having our model predict log(sigma^2)
    rather than sigma^2. 
    """
    
    y_true = y_true.reshape(-1)
    mu = y_pred[:, 0]
    si = y_pred[:, 1]
    loss = (si + (y_true - mu) ** 2/torch.exp(si)) / 2.0
    return torch.mean(loss, dim=0)

def train_differential(model, X_train, y_train, X_val, y_val, 
        epochs=1000, model_save_path=None,
        lr=1e-4, opt='adam', writer=None):
    if opt == "adam":
        optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    elif opt == "sgd":
        optimizer = torch.optim.SGD(model.parameters(), lr=lr)
    else:
        raise ValueError("Optimizer choice unknown")
    scheduler = optim.lr_scheduler.ExponentialLR(optimizer, gamma=0.999)
    loss_func = gaussian_nll

    losses = []

    for epoch in range(epochs):
    
        best_val_loss = float('inf')
        
        prediction = model(X_train)
        # print(X_train[0], y_train[0], prediction[0])
        loss = loss_func(prediction, y_train)

        optimizer.zero_grad()
        loss.backward()         
        optimizer.step()
    #     scheduler.step()
        if epoch % 10 == 0 and writer is not None:
            writer.add_scalar('train_loss', loss, global_step=epoch)
        print(f'epoch number: {epoch+1}, MSE Loss: {loss.data}')
        
        if epoch % 100 == 0:
            val_y_preds = model(X_val)
            val_loss = loss_func(val_y_preds, y_val)
            if writer is not None:
                writer.add_scalar('validation_loss', val_loss, global_step=epoch)
            print('Validation Loss: ', val_loss.data)
            losses.append(val_loss.data.item())
            if val_loss.data < best_val_loss and model_save_path is not None:
                best_val_loss = val_loss.data
                torch.save(model.state_dict(), model_save_path)


def load_initial_models(ensemble_size=5, input_dim=2, output_dim=2, hidden_dim=64):
    models = []
    for i in range(ensemble_size):
        model = ActiveNet(input_dim=input_dim, output_dim=output_dim)
        models.append(model)
    return models

def predict_with_uncertainty(models, x):
    '''
    Args:
        models: The trained pytorch model ensemble
        x: the input tensor with shape [N, M]
        samples: the number of monte carlo samples to collect
    Returns:
        y_mean: The expected value of our prediction
        y_std: The standard deviation of our prediction
    '''
    mus_arr = []
    sigs_arr = []

    x = x.reshape()

    for model in models:
        y_pred = model(x)
        mu = y_pred[:, 0]
        si = y_pred[:, 1]

        mus_arr.append(mu)
        sigs_arr.append(si)

    mu_arr = np.array(mu_arr)
    si_arr = np.array(si_arr)
    var_arr = np.exp(si_arr)

    y_mean = np.mean(mu_arr, axis=0)
    y_variance = np.mean(var_arr + mu_arr**2, axis=0) - y_mean**2
    y_std = np.sqrt(y_variance)
    return y_mean, y_std

def train_val_test_split(X, y, train_ratio=0.7, val_ratio=0.2, test_ratio=0.1, random_seed=54):
    size = len(X)
    train_num = int(size*train_ratio)
    val_num = int(size*val_ratio)
    
    # randomly shuffling X and y in unison
    # np.random.seed(random_seed)
    perm = np.random.permutation(len(X))
    X = X[perm]
    y = y[perm]
    
    X_train, X_val, X_test = X[:train_num], X[train_num:train_num+val_num], X[train_num+val_num:]
    y_train, y_val, y_test = y[:train_num], y[train_num:train_num+val_num], y[train_num+val_num:]
    
    return X_train, X_val, X_test, y_train, y_val, y_test
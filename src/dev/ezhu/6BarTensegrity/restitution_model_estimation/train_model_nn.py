import tensorflow as tf
import numpy as np
import math
import time
import preprocessing
import argparse
from dynamics_auto_uc import NNDynamicsModel_Auto_UC
import matplotlib.pyplot as plt

def compute_normalization(data):

    print('Computing normalization statistics...')
    obs_t = data['obs_t']
    obs_tp1 = data['obs_tp1']
    deltas = obs_tp1-obs_t

    mean_obs = np.mean(obs_t,axis=0)
    std_obs = np.std(obs_t,axis=0)
    mean_obs_tp1 = np.mean(obs_tp1,axis=0)
    std_obs_tp1 = np.std(obs_tp1,axis=0)
    mean_deltas = np.mean(deltas,axis=0)
    std_deltas = np.std(deltas,axis=0)

    normalization = {'mean_obs':mean_obs, 'std_obs':std_obs,
                    'mean_obs_tp1':mean_obs_tp1, 'std_obs_tp1':std_obs_tp1,
                    'mean_deltas':mean_deltas, 'std_deltas':std_deltas}
    return normalization

def train(args, data, sess):

    # Extract training data
    x_train = data['x_train']['features']
    y_train = data['y_train']['labels']

    data_new = {'obs_t':x_train, 'obs_tp1':y_train}

    x_dim = x_train.shape[1]
    y_dim = y_train.shape[1]
    n_data = x_train.shape[0]

    normalization = compute_normalization(data_new)

    dyn_model = NNDynamicsModel_Auto_UC(x_dim,y_dim,args.n_layers,args.size,tf.tanh,None,
        normalization,args.batch_size,args.n_iters,args.learning_rate,sess)

    dyn_model.fit(data_new)

    return dyn_model

def test(args, paths, dyn_model, sess):

    s_k = np.zeros((1,6))

    for i in range(len(paths)):
        data = preprocessing.paths_to_array(paths[i])

        x_data = data['obs_t']
        y_data = data['obs_tp1']

        x_data, y_data, _, _ = preprocessing.process_data(x_data,y_data)

        x_dim = x_data.shape[1]
        y_dim = y_data.shape[1]
        n_data = x_data.shape[0]

        s_k = s_k + x_data[0,:].reshape((1,x_dim))/len(paths)

        for j in range(y_dim):
            plt.figure(j)
            plt.plot(np.arange(n_data),y_data[:,j],'k',alpha=0.5)

    for i in range(n_data):
        # s_k = x_data[i,:].reshape((1,x_dim))
        s_kp1_samp, s_kp1_mean, std = dyn_model.predict(s_k)

        if i == 0:
            y_pred_samp = s_kp1_samp.reshape((1,y_dim))
            y_pred_mean = s_kp1_mean.reshape((1,y_dim))
            std_vec = std.reshape((1,y_dim))
        else:
            y_pred_samp = np.append(y_pred_samp,s_kp1_samp.reshape((1,y_dim)),axis=0)
            y_pred_mean = np.append(y_pred_mean,s_kp1_mean.reshape((1,y_dim)),axis=0)
            std_vec = np.append(std_vec,std.reshape((1,y_dim)),axis=0)

        s_k = s_kp1_mean

    for i in range(y_dim):
        plt.figure(i)
        # plt.plot(np.arange(n_data),y_data[:,i],'k')
        # plt.plot(np.arange(n_data),y_pred_samp[:,i],'b')
        plt.plot(np.arange(n_data),y_pred_mean[:,i],'b')
        plt.plot(np.arange(n_data),y_pred_mean[:,i]+2*std_vec[:,i],'r--')
        plt.plot(np.arange(n_data),y_pred_mean[:,i]-2*std_vec[:,i],'r--')

        if (i == 0):
            plt.title('X position')
        elif (i == 1):
            plt.title('Y position')
        elif (i == 2):
            plt.title('Z position')
        elif (i == 3):
            plt.title('X velocity')
        elif (i == 4):
            plt.title('Y velocity')
        elif (i == 5):
            plt.title('Z velocity')

        plt.draw()

    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='train')
    parser.add_argument('--n_layers', type=int, default=3)
    parser.add_argument('--size', type=int, default=64)
    parser.add_argument('--learning_rate', type=float, default=5e-3)
    parser.add_argument('--n_iters', type=int, default=10)
    parser.add_argument('--batch_size', type=int, default=500)
    parser.add_argument('--seed', type=int, default=3)
    args = parser.parse_args()

    # Set seed
    np.random.seed(args.seed)
    tf.set_random_seed(args.seed)

    n_train = 500000
    n_test = 0
    n_data = n_train+n_test

    paths = preprocessing.get_paths(n_data,full_data=True,zero_x=True)
    print('Number of paths loaded: ', len(paths))
    data = preprocessing.get_train_test_sets(n_train,n_test,paths[0:-2],shuf=True)

    test_path = paths[-200:-1]

    sess = tf.Session()

    if args.mode == 'train':
        print('Train mode')
        dyn_model = train(args,data,sess)
        test(args,test_path,dyn_model,sess)
    elif args.mode == 'test':
        print('Test mode')
        test(args,data,sess)

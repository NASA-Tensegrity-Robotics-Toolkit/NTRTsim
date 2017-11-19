# Library of functions for preprocessing data for estimation of the restitution model

import numpy as np
import os
import os.path

def shuffle(a, o):
    indicies = np.arange(o.shape[0])
    np.random.shuffle(indicies)

    a_shuf = a[indicies,:]
    o_shuf = o[indicies,:]

    # print('Data shuffling done')

    return a_shuf, o_shuf

def standardize(X):
    mean = np.mean(X, axis=0)
    std = np.std(X, axis=0)+1e-5

    X_std = np.divide((X-np.tile(mean, (X.shape[0],1))),np.tile(std, (X.shape[0],1)))

    # print('standardize done...')

    return X_std, mean, std

def read_csv_data(filename, full_data=False):
    success = True

    with open(filename,'r') as f_in:
        rows = sum(1 for row in f_in)

    if rows == 1:
        print('Empty file, skipping...')
        success = False
        path = []
        return path
    elif rows == 2:
        print('Data incomplete, skipping...')
        success = False
        path = []
        return path

    data_tmp = np.loadtxt(filename,delimiter=',',dtype=None,skiprows=1)
    # print(filename)
    # print(data_tmp.shape)
    if not full_data:
        # Only save state in (first state) and state out (last state) information
        x_tmp = data_tmp[0,]
        x_tmp = x_tmp.reshape((1,len(x_tmp)))
        y_tmp = data_tmp[-1,]
        y_tmp = y_tmp.reshape((1,len(y_tmp)))
    else:
        # Save full dynamics trajectory x is s_k, y is s_kp1
        x_tmp = data_tmp[0:-2,]
        y_tmp = data_tmp[1:-1,]

    path = {'observations':x_tmp,'next_observations':y_tmp}

    return path

def process_data(x_data, y_data):
    sf = 10

    # With all pos and all vels
    x_feat = x_data[:,1:7]/sf
    # With x, y, z pos and x, z slopes
    # x_feat = np.delete(x_data,[0,7,8,11,12,13],1)
    # x_feat[:,0:6] = x_feat[:,0:6]/sf

    # delta labels
    # y_feat = (y_data[:,1:7]-x_data[:,1:7])/sf
    # state labels
    y_feat = y_data[:,1:7]/sf
    x_state = x_data[:,1:7]/sf
    y_state = y_data[:,1:7]/sf

    return x_feat, y_feat, x_state, y_state

def get_paths(n_data, full_data=False):
    print('Loading data into paths...')
    directory = '../../../../../../Documents/data/'
    paths = []
    datapoints = 0

    n_files = int(np.round(len(os.listdir(directory))/2))
    file_num_list = np.arange(n_files)

    # print(n_files)

    while True:
        file_num = np.random.choice(file_num_list,1)[0]
        filename = directory+str(file_num)+'_Response.csv'
        path = read_csv_data(filename,full_data)
        if len(path) > 0:
            paths.append(path)
            datapoints += path['observations'].shape[0]

        if datapoints >= n_data:
            break

        idx = np.nonzero(file_num_list==file_num)
        file_num_list = np.delete(file_num_list,idx)

    return paths

def get_train_test_sets(n_train, n_test, paths, shuf=True):
    print('Creating training and test sets...')

    data = paths_to_array(paths)
    x_data = data['obs_t']
    y_data = data['obs_tp1']

    if shuf:
        x_data, y_data = shuffle(x_data, y_data)

    x_feat, y_feat, x_state, y_state = process_data(x_data,y_data)

    x_train = {'features':x_feat[0:n_train], 'states':x_state[0:n_train]}
    y_train = {'labels':y_feat[0:n_train], 'states':y_state[0:n_train]}
    x_test = {'features':x_feat[n_train:], 'states':x_state[n_train:]}
    y_test = {'labels':y_feat[n_train:], 'states':y_state[n_train:]}

    data = {'x_train':x_train, 'y_train':y_train, 'x_test':x_test, 'y_test':y_test, 'x_state':x_state, 'y_state':y_state}

    return data

def paths_to_array(paths):
    print('Converting paths to dataset...')

    if type(paths) == dict:
        obs_t = paths['observations']
        obs_tp1 = paths['next_observations']
    else:
        for i in range(len(paths)):
            if i == 0:
                obs_t = paths[i]['observations']
                obs_tp1 = paths[i]['next_observations']
            else:
                obs_t = np.append(obs_t,paths[i]['observations'],axis=0)
                obs_tp1 = np.append(obs_tp1,paths[i]['next_observations'],axis=0)

    n_data = obs_t.shape[0]
    data = {'obs_t':obs_t, 'obs_tp1':obs_tp1}
    print('Dataset built with %i datapoints' % n_data)
    return data

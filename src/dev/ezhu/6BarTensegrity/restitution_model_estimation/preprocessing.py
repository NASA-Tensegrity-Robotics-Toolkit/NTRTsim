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

def read_csv_data(filename, x_data, y_data, full_data=False):
    success = True

    with open(filename,'r') as f_in:
        rows = sum(1 for row in f_in)

    if rows == 1:
        print('Empty file, skipping...')
        success = False
        return x_data, y_data
    elif rows == 2:
        print('Data incomplete, skipping...')
        success = False
        return x_data, y_data

    data_tmp = np.loadtxt(filename,delimiter=',',dtype=None,skiprows=1)

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

    if len(x_data) == 0:
        x_data = x_tmp
        y_data = y_tmp
    else:
        x_data = np.append(x_data,x_tmp,axis=0)
        y_data = np.append(y_data,y_tmp,axis=0)

    return x_data, y_data

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

def get_dataset(n_train, n_test, shuf=True, full_data=False):
    print('Loading data sets')
    directory = '../../../../../../Documents/data/'
    x_data = []
    y_data = []

    n_data = n_train + n_test
    n_files = int(np.round(len(os.listdir(directory))/2))
    file_num_list = np.arange(n_files)

    while True:
        file_num = np.random.choice(file_num_list,1)[0]
        filename = directory+str(file_num)+'_Response.csv'
        x_data, y_data = read_csv_data(filename,x_data,y_data,full_data)
        if x_data.shape[0] >= n_data:
            break

        idx = np.nonzero(file_num_list==file_num)
        file_num_list = np.delete(file_num_list,idx)

    x_data = x_data[0:n_data]
    y_data = y_data[0:n_data]

    if shuf:
        x_data, y_data = shuffle(x_data,y_data)

    x_feat, y_feat, x_state, y_state = process_data(x_data,y_data)

    x_train = {'features':x_feat[0:n_train], 'states':x_state[0:n_train]}
    y_train = {'labels':y_feat[0:n_train], 'states':y_state[0:n_train]}
    x_test = {'features':x_feat[n_train:], 'states':x_state[n_train:]}
    y_test = {'labels':y_feat[n_train:], 'states':y_state[n_train:]}

    data = {'x_train':x_train, 'y_train':y_train, 'x_test':x_test, 'y_test':y_test, 'x_state':x_state, 'y_state':y_state}
    print('Load data done...')

    return data

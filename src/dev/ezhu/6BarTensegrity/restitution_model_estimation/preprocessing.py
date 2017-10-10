# Library of functions for preprocessing data for estimation of the restitution model

import numpy as np

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

def read_csv_data(filename, x_train, y_train):
    sf = 10
    data_tmp = np.loadtxt(filename,delimiter=',',dtype=None,skiprows=1)
    x_tmp = np.concatenate((data_tmp[0,2]*sf,data_tmp[0,4:7]*sf,data_tmp[0,9:]))
    y_tmp = sf*np.concatenate((data_tmp[-1,1]-data_tmp[0,1],data_tmp[-1,2],data_tmp[-1,3]-data_tmp[0,3],data_tmp[-1,4:7]))
    if len(x_train) == 0:
        x_train = np.append(x_train,x_tmp,axis=0)
        y_train = np.append(y_train,y_tmp,axis=0)
        x_train = np.reshape(x_train,(1,len(x_train)))
        y_train = np.reshape(y_train,(1,len(y_train)))
    else:
        x_train = np.append(x_train,[x_tmp],axis=0)
        y_train = np.append(y_train,[y_tmp],axis=0)

    return x_train, y_train

def get_data_sets():
    print('Loading data sets')
    directory = '../../../../../build/dev/ezhu/6BarTensegrity/data/'
    x_data = []
    y_data = []
    n_data = 5001
    n_train = 4001
    n_test = n_data - n_train
    for i in range(0,n_data):
        filename = directory+str(i)+'_Response.csv'
        x_data, y_data = read_csv_data(filename,x_data,y_data)

    x_data, y_data = shuffle(x_data,y_data)
    x_train = x_data[0:n_train]
    y_train = y_data[0:n_train]
    x_test = x_data[n_train:]
    y_test = y_data[n_train:]
    return x_train, y_train, x_test, y_test

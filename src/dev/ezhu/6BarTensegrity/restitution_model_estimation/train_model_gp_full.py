import tensorflow as tf
import numpy as np
import math
import time
import preprocessing
import gpflow as gp
import matplotlib.pylab as plt

def main():
    data = preprocessing.get_dataset(n_train=500,n_test=100,shuf=True,full_data=False)
    x_train = data['x_train']['features']
    y_train = data['y_train']['labels']
    x_test = data['x_test']['features']
    y_test = data['y_test']['labels']

    x_dim = x_train.shape[1]
    y_dim = y_train.shape[1]
    n_train = x_train.shape[0]
    n_test = x_test.shape[0]

    # Build model
    print('Building model...')

    to_train = [0,1,2,3,4,5]

    k = gp.kernels.RBF(input_dim=len(to_train),variance=1,lengthscales=1)
    # meanf = gp.mean_functions.Linear(1,0)
    meanf = gp.mean_functions.Zero()
    # likelihood = gp.likelihoods.Gaussian()
    gp_models = []

    m_full = gp.gpr.GPR(x_train[:,to_train],y_train[:,to_train],kern=k,mean_function=meanf)

    # Optimize
    print('Optimizing model...')
    m_full.optimize()
    print(m_full)

    mean_y1,var_y1 = m_full.predict_y(x_train[:,to_train])
    mean_f1,var_f1 = m_full.predict_f(x_train[:,to_train])
    _,cov_f1 = m_full.predict_f_full_cov(x_train[:,to_train])

    print(cov_f1.shape)
    
    for i in range(len(to_train)):
        x = x_train[:,to_train[i]]
        y = y_train[:,to_train[i]]
        x_min = np.min(x)
        x_max = np.max(x)
        buff = (x_max-x_min)*0.1
        x_lin = np.linspace(x_min,x_max,100).reshape((100,1))

        idx = np.argsort(x)
        x_train = x_train[idx]
        y_train = y_train[idx]

        plt.figure()
        plt.plot(x,y,'kx',mew=2)

        plt.plot(x_train[:,to_train[i]],mean_y1[:,i],'b',lw=2)
        plt.plot(x_train[:,to_train[i]],mean_y1[:,i]-2*np.sqrt(var_y1[:,i]),'r',lw=2)
        plt.plot(x_train[:,to_train[i]],mean_y1[:,i]+2*np.sqrt(var_y1[:,i]),'r',lw=2)


        plt.plot(x_train[:,to_train[i]],mean_f1[:,i],'b-.',lw=2)
        plt.plot(x_train[:,to_train[i]],mean_f1[:,i]-2*np.sqrt(var_f1[:,i]),'r-.',lw=2)
        plt.plot(x_train[:,to_train[i]],mean_f1[:,i]+2*np.sqrt(var_f1[:,i]),'r-.',lw=2)

        plt.xlabel('state '+str(to_train[i])+' in')
        plt.ylabel('state '+str(to_train[i])+' out')

        # plt.figure()
        # plt.plot(x_lin,var_y1,'b',lw=2)
        # plt.plot(x_lin,var_f1,'r',lw=2)

        plt.draw()

if __name__ == '__main__':
    main()

    plt.show()

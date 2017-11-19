import tensorflow as tf
import numpy as np
import math
import time
import preprocessing
import gpflow as gp
import matplotlib.pylab as plt

def main():
    n_train = 1000
    n_test = 200
    n_data = n_train + n_test
    paths = preprocessing.get_paths(n_data,full_data=False)
    data = preprocessing.get_train_test_sets(n_train,n_test,paths,shuf=True)

    x_train = data['x_train']['features']
    y_train = data['y_train']['labels']
    x_test = data['x_test']['features']
    y_test = data['y_test']['labels']

    x_dim = x_train.shape[1]
    y_dim = y_train.shape[1]
    n_train = x_train.shape[0]
    n_test = x_test.shape[0]

    # Process data to zero x, y, and z positions
    x_0_train = x_train[:,0].reshape((n_train,1))
    y_0_train = x_train[:,1].reshape((n_train,1))
    z_0_train = x_train[:,2].reshape((n_train,1))

    x_0_test = x_test[:,0].reshape((n_test,1))
    y_0_test = x_test[:,1].reshape((n_test,1))
    z_0_test = x_test[:,2].reshape((n_test,1))

    x_train_new = x_train - np.concatenate((x_0_train,y_0_train,z_0_train,np.zeros((n_train,3))),axis=1)
    y_train_new = y_train - np.concatenate((x_0_train,y_0_train,z_0_train,np.zeros((n_train,3))),axis=1)

    x_test_new = x_test - np.concatenate((x_0_test,y_0_test,z_0_test,np.zeros((n_test,3))),axis=1)
    y_test_new = y_test - np.concatenate((x_0_test,y_0_test,z_0_test,np.zeros((n_test,3))),axis=1)

    # Build model
    print('Building model...')

    to_train = [0,1,2,3,4,5]

    # k = gp.kernels.RBF(input_dim=len(to_train),variance=1,lengthscales=1)
    k = gp.kernels.Matern52(input_dim=len(to_train),lengthscales=1)
    # meanf = gp.mean_functions.Linear(1,0)
    meanf = gp.mean_functions.Zero()
    # likelihood = gp.likelihoods.Gaussian()
    gp_models = []

    m_full = gp.gpr.GPR(x_train[:,to_train],y_train[:,to_train],kern=k,mean_function=meanf)
    # m_full = gp.gpr.GPR(x_train_new[:,to_train],y_train_new[:,to_train],kern=k,mean_function=meanf)

    # Optimize
    print('Optimizing model...')
    m_full.optimize()
    print(m_full)

    mean_y1,var_y1 = m_full.predict_y(x_test[:,to_train])
    mean_f1,var_f1 = m_full.predict_f(x_test[:,to_train])
    _,cov_f1 = m_full.predict_f_full_cov(x_test[:,to_train])

    print(cov_f1.shape)

    for i in to_train:
        x = x_test[:,i]
        y = y_test[:,i]

        idx = np.argsort(x)
        x = x[idx]
        y = y[idx]

        plt.figure()
        plt.plot(x,y,'ko',mew=2)

        plt.plot(x,mean_y1[idx,i],'b',lw=2)
        plt.plot(x,mean_y1[idx,i]-2*np.sqrt(var_y1[idx,i]),'r',lw=2)
        plt.plot(x,mean_y1[idx,i]+2*np.sqrt(var_y1[idx,i]),'r',lw=2)


        plt.plot(x,mean_f1[idx,i],'b-.',lw=2)
        plt.plot(x,mean_f1[idx,i]-2*np.sqrt(var_f1[idx,i]),'r-.',lw=2)
        plt.plot(x,mean_f1[idx,i]+2*np.sqrt(var_f1[idx,i]),'r-.',lw=2)

        plt.xlabel('Touchdown state')
        plt.ylabel('Liftoff state')

        if i == 0:
            plt.title('X position')
        elif i == 1:
            plt.title('Y position')
        elif i == 2:
            plt.title('Z position')
        elif i == 3:
            plt.title('X velocity')
        elif i == 4:
            plt.title('Y velocity')
        elif i == 5:
            plt.title('Z velocity')

        plt.draw()

if __name__ == '__main__':
    main()

    plt.show()

import tensorflow as tf
import numpy as np
import math
import time
import preprocessing
import gpflow as gp
import matplotlib.pylab as plt

def distance(X, Xp):
    if np.isscalar(X):
        n = 1
        m = 1

        XX = X**2
        XpXp = Xp**2
        XXp = X*Xp
    else:
        n = X.shape[0]
        m = Xp.shape[0]

        XX = np.tile(np.sum(np.multiply(X,X),axis=1).reshape((n,1)),(1,m))
        XpXp = np.tile(np.sum(np.multiply(Xp,Xp),axis=1).reshape((1,m)),(n,1))
        XXp = X.dot(Xp.T)

    dist = XX-2*XXp+XpXp

    return dist

def se_kernel(X, Xp, params):
    sigma = params[0]
    l = params[1]
    dist = distance(X,Xp)
    K = sigma**2*np.exp(-np.divide(dist,(2*l**2)))
    return K

def rq_kernel(X, Xp, params):
    sigma = params[0]
    l = params[1]
    a = params[2]
    dist = distance(X,Xp)
    K = sigma**2*(1+np.divide(dist,(2*a*(l**2))))**(-a)
    return K

def conditional(kernel, params, x_new, x_train, y_train):
    n = x_train.shape[1]
    m = y_train.shape[1]

    K_x = kernel(x_train,x_train,params)
    K_x_new = kernel(x_new,x_new,params)
    K_x_new_x = kernel(x_new,x_train,params)
    inv_K_x = np.linalg.inv(K_x)

    m_x_new = np.mean(x_new,axis=0).reshape((n,1))
    m_y_train = np.mean(y_train,axis=0).reshape((m,1))
    mu = m_x_new + K_x_new_x.dot(inv_K_x).dot(y_train.T-m_y_train)
    sigma = K_x_new - K_x_new_x.dot(inv_K_x.dot(K_x_new_x.T))

    return mu, sigma

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

    print(x_train.shape)

    # plt.figure()
    # plt.plot(x_train[:,0],y_train[:,0],'kx',mew=2)
    # plt.xlabel('x_in')
    # plt.ylabel('x_out')
    # plt.figure()
    # plt.plot(x_train[:,1],y_train[:,1],'kx',mew=2)
    # plt.xlabel('y_in')
    # plt.ylabel('y_out')
    # plt.figure()
    # plt.plot(x_train[:,2],y_train[:,2],'kx',mew=2)
    # plt.xlabel('z_in')
    # plt.ylabel('z_out')
    # plt.figure()
    # plt.plot(x_train[:,3],y_train[:,3],'kx',mew=2)
    # plt.xlabel('vx_in')
    # plt.ylabel('vx_out')
    # plt.figure()
    # plt.plot(x_train[:,4],y_train[:,4],'kx',mew=2)
    # plt.xlabel('vy_in')
    # plt.ylabel('vy_out')
    # plt.figure()
    # plt.plot(x_train[:,5],y_train[:,5],'kx',mew=2)
    # plt.xlabel('vz_in')
    # plt.ylabel('vz_out')
    # plt.draw()

    # Build model
    print('Building model...')

    k1 = gp.kernels.RBF(input_dim=1,variance=1,lengthscales=1)
    k2 = gp.kernels.RBF(input_dim=x_dim,variance=1,lengthscales=1)
    # meanf = gp.mean_functions.Linear(1,0)
    meanf = gp.mean_functions.Zero()
    # likelihood = gp.likelihoods.Gaussian()
    gp_models = []

    m_full = gp.gpr.GPR(x_train,y_train,kern=k2,mean_function=meanf)

    # to_train = [3]
    to_train = [0,1,2,3,4,5]

    for i in to_train:
        x = x_train[:,i].reshape((n_train,1))
        y = y_train[:,i].reshape((n_train,1))
        m = gp.gpr.GPR(x,y,kern=k1,mean_function=meanf)
        gp_models.append(m)

    # m1.likelihood.variance = 1
    # m2 = gp.gpr.GPR(x_train[:,state].reshape((n_train,1)),y_train[:,state].reshape((n_train,1)),kern=k2)
    # m2.likelihood.variance = 0.01

    # m1 = gp.gpmc.GPMC(x_train[:,state].reshape((n_train,1)),y_train[:,state].reshape((n_train,1)),k1,likelihood)
    # m1.likelihood.variance = 0.01
    # m2 = gp.gpmc.GPMC(x_train[:,state].reshape((n_train,1)),y_train[:,state].reshape((n_train,1)),k2,likelihood)
    # m2.likelihood.variance = 0.01

    # Optimize
    print('Optimizing model...')
    for i in range(len(gp_models)):
        m = gp_models[i]
        m.optimize()
        print(m)

        x = x_test[:,to_train[i]]
        y = y_test[:,to_train[i]]
        x_min = np.min(x)
        x_max = np.max(x)
        buff = (x_max-x_min)*0.1
        x_lin = np.linspace(x_min-buff,x_max+buff,100).reshape((100,1))

        idx = np.argsort(x)
        x = x[idx].reshape((n_test,1))
        y = y[idx].reshape((n_test,1))

        mean_y1,var_y1 = m.predict_y(x_lin)

        print(var_y1)
        mean_f1,var_f1 = m.predict_f(x_lin)
        _,cov_f1 = m.predict_f_full_cov(x_lin)

        print(cov_f1.shape)

        plt.figure()
        plt.plot(x,y,'ko',mew=2)

        plt.plot(x_lin,mean_y1,'b',lw=2)
        plt.plot(x_lin,mean_y1-2*np.sqrt(var_y1),'r',lw=2)
        plt.plot(x_lin,mean_y1+2*np.sqrt(var_y1),'r',lw=2)


        # plt.plot(x_lin,mean_f1,'b-.',lw=2)
        # plt.plot(x_lin,mean_f1-2*np.sqrt(var_f1),'r-.',lw=2)
        # plt.plot(x_lin,mean_f1+2*np.sqrt(var_f1),'r-.',lw=2)

        plt.xlabel('Touchdown state')
        plt.ylabel('Liftoff state')

        if to_train[i] == 0:
            plt.title('X position [m]')
        elif to_train[i] == 1:
            plt.title('Y position [m]')
        elif to_train[i] == 2:
            plt.title('Z position [m]')
        elif to_train[i] == 3:
            plt.title('X velocity [m/s]')
        elif to_train[i] == 4:
            plt.title('Y velocity [m/s]')
        elif to_train[i] == 5:
            plt.title('Z velocity [m/s]')

        # plt.figure()
        # plt.plot(x_lin,var_y1,'b',lw=2)
        # plt.plot(x_lin,var_f1,'r',lw=2)

        plt.draw()

    # mean_y,var_y = m.predict_y(x_test)
    # mean_f,var_f = m.predict_f(x_test)
    # mean_y1,var_y1 = m1.predict_y(x_train[:,state].reshape((n_train,1)))
    # mean_f1,var_f1 = m1.predict_f(x_train[:,state].reshape((n_train,1)))
    # mean_y1,var_y1 = m1.predict_y(x_lin)
    # mean_f1,var_f1 = m1.predict_f(x_lin)
    # mean_y2,var_y2 = m2.predict_y(x_test[:,state].reshape((n_test,1)))

    # print(m1.sorted_params)

if __name__ == '__main__':
    main()

    plt.show()

import numpy as np
import preprocessing
import matplotlib.pyplot as plt

if __name__ == '__main__':

    # n_train = 50000
    n_train = 50000
    # n_test = 5000
    n_test = 0
    n_data = n_train+n_test

    paths = preprocessing.get_paths(n_data,full_data=True)

    print(len(paths))
    # print(paths[1]['observations'])

    for i in range(len(paths)):
        obs = paths[i]['observations']

        impact_vel = obs[0,5]/10
        impact_y = obs[0,2]/10

        min_idx = np.argmin(obs[:,2])
        min_y = obs[min_idx,2]/10
        delta_y = min_y-impact_y
        percent_change = 100*delta_y/impact_y

        if i == 0:
            data = np.array([[impact_vel,percent_change]])
        else:
            data = np.append(data,[[impact_vel,percent_change]],axis=0)

    plt.figure()
    plt.plot(data[:,0],data[:,1],'.')
    plt.xlabel('Vertical Impact Velocity [m/s]')
    plt.ylabel('Percent Vertical CoM Position Change [%]')

    plt.draw()
    plt.show()

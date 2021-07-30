#!/usr/bin/python3

# Script for plotting NTRT simulation results of belka walking trajectories.
# Andrew Sabelhaus 20221

# from the environment
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib

def plot_from_file(filename, start_row=2):
    all_data = np.genfromtxt(filename, delimiter=',', skip_header=(start_row))
    body_to_plot = 3
    bdy_start = 1 + 7*body_to_plot
    time = all_data[:,0]
    x = all_data[:, bdy_start]
    y = all_data[:, bdy_start+1]
    z = all_data[:, bdy_start+2]
    yaw = all_data[:, bdy_start+3]
    pit = all_data[:, bdy_start+4]
    rol = all_data[:, bdy_start+5]
    # ensure the robot starts at 0,0 and moves in a positive direction (in x)
    x = -(x - x[0])
    z = z - z[0]
    fig, ax = plt.subplots(1,figsize=[7.5,5])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.plot(x, z, color='r', linestyle='-', linewidth=2)
    # ax.tick_params(axis='y', labelcolor=color_angle)
    # ax.legend([r'$\theta$'])
    ax.set(title='Locomotion of the Tensegrity Quadruped')
    fig.subplots_adjust(top=0.880, bottom=0.140, left=0.110, right=0.900, hspace=0.250)
    plt.show()

def plot_multiple_files(f1, f2, f3, start_row=2):
    timept1 = 10.0
    timept2 = 28.0
    titlefontsize = 12
    axisfontsize = 9
    dat1 = np.genfromtxt(f1, delimiter=',', skip_header=(start_row))
    dat2 = np.genfromtxt(f2, delimiter=',', skip_header=(start_row))
    dat3 = np.genfromtxt(f3, delimiter=',', skip_header=(start_row))
    body_to_plot = 1
    bdy_start = 1 + 7*body_to_plot
    bdy2_plot = 3
    bdy2_start = 1 + 7*bdy2_plot
    bdy3_plot = 0
    bdy3_start = 1 + 7*bdy3_plot
    t1 = dat1[:,0]
    x1 = dat1[:, bdy_start]
    z1 = dat1[:, bdy_start+2]
    t2 = dat2[:,0]
    x2 = dat2[:, bdy2_start]
    z2 = dat2[:, bdy2_start+2]
    t3 = dat3[:,0]
    x3 = dat3[:, bdy3_start]
    z3 = dat3[:, bdy3_start+2]
    # ensure the robot starts at 0,0 and moves in a positive direction (in x)
    x1 = -(x1 - x1[0])
    z1 = z1 - z1[0]
    x2 = -(x2 - x2[0])
    z2 = z2 - z2[0]
    x3 = -(x3 - x3[0])
    z3 = z3 - z3[0]
    # timepoints should be same for all runs, since constant sampling rate
    tp1_idx = np.where(t1 >= timept1)[0][0]
    # plotting
    fig, ax = plt.subplots(1,figsize=[6,4])
    ax.set_xlabel('X (m)', fontsize=axisfontsize)
    ax.set_ylabel('Y (m)', fontsize=axisfontsize)
    ax.plot(x1, z1, linestyle='-', linewidth=2)
    ax.plot(x2, z2, linestyle='-', linewidth=2)
    ax.plot(x3, z3, linestyle='-', linewidth=2)
    ax.scatter(x1[tp1_idx], z1[tp1_idx], marker='x', c='k', s=50, zorder=1)
    ax.scatter(x2[tp1_idx], z2[tp1_idx], marker='x', c='k', s=50, zorder=1)
    ax.scatter(x3[tp1_idx], z3[tp1_idx], marker='x', c='k', s=50, zorder=1)
    ax.grid(True)
    ax.set_ylim([-15, 15])
    # ax.tick_params(axis='y', labelcolor=color_angle)
    ax.legend(['BDAC Gait', 'ACBD Gait', 'Alternating Gait'], fontsize=axisfontsize)
    plt.title('Locomotion of the Tensegrity Quadruped', fontsize=titlefontsize)
    fig.subplots_adjust(top=0.880, bottom=0.150, left=0.140, right=0.940, hspace=0.250)
    plt.show()


def get_hardcoded_filename():
    # filename = 'data_analysis/walking_results/AppBelkaWalking_bdac_longer_20deg_2020-12-31.txt'
    filename = 'data_analysis/walking_results/AppBelkaWalking_acbd_longer_20deg_2020-12-31.txt'
    # filename = 'data_analysis/walking_results/AppBelkaWalking_withmirror_20deg_2020-12-4.txt'
    return filename

def get_hardcoded_filenames():
    f1 = 'data_analysis/walking_results/AppBelkaWalking_bdac_longer_20deg_2020-12-31.txt'
    f2 = 'data_analysis/walking_results/AppBelkaWalking_acbd_longer_20deg_2020-12-31.txt'
    f3 = 'data_analysis/walking_results/AppBelkaWalking_withmirror_20deg_2020-12-4.txt'
    return (f1,f2,f3)

if __name__ == '__main__':
    # filename = get_hardcoded_filename()
    (f1, f2, f3) = get_hardcoded_filenames()
    # plot_from_file(filename, start_row=2)
    plot_multiple_files(f1, f2, f3)
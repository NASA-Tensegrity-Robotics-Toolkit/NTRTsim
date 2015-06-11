#!/opt/anaconda/bin/python
# -*- coding: utf-8 -*-

import subprocess
import os
import argparse
import csv
from operator import itemgetter
import numpy as np
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

ntrtDir="/home/axydes/Dropbox/School/Tensegrity/NTRTsim/"
exDir=ntrtDir+"build/dev/axydes/DuCTT/"
rsrcDir="axydes/DuCTT/posRobustTest/"
fullRsrcDir=ntrtDir+"resources/src/"+rsrcDir
ex=exDir+"AppDuCTTLearn"
numEp=20
pickleFile='logs/allscores.pickle'
exParams=["-c","2","-d","--rot_y","1","-a","45","--duct_axis","1","-r",rsrcDir,"-s","60000","-e",str(numEp)]

parser = argparse.ArgumentParser(description='DuCTT learned parameters robustness testing.')
parser.add_argument('-m', dest='method', type=int, default=3,
        help='Method to test (1-4), default=3: (1) Position, (2) Duct Width, (3) Plot position results, or (4) Plot duct width results')
parser.add_argument('-r', dest='rsrcDir', type=str, default='axydes/DuCTT/posRobustTest/',
        help='Resource directory to use (subdir of resources/src)')
parser.add_argument('-e', dest='numEp', type=int, default=20,
        help='Number of episodes to run for each call')
parser.add_argument('-x', dest='ex', type=str, default='AppDuCTTLearn',
        help='Executable to run')
parser.add_argument('-p', dest='pfile', type=str, default='logs/allscores.pickle',
        help='Pickle file to plot results from')

def saveLoad(opt, filename, varArr):
    if opt == "save":
        print 'saving data'
        with open(fullRsrcDir+filename,'wb') as f:
            for var in varArr:
                pickle.dump(var,f)
        print 'data saved'
    elif opt == "load":
        print 'loading data'
        newVars=[]
        with open(fullRsrcDir+filename,'rb') as f:
            for var in varArr:
                newVars.append(pickle.load(f))
            varArr = newVars
        print 'data loaded'
    else:
        print 'Invalid saveLoad option'

    return varArr

def loadScores(folder):
    scoresFile=folder+"logs/scores.csv"
    unsorted=[]

    with open(scoresFile, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        for row in reader:
            score = []
            score.append(float(row[0]))
            score.append(float(row[1]))
            unsorted.append(score)

    os.remove(scoresFile)

    return unsorted

def getScores(scoreArr):
    score1 = []
    score2 = []

    for entry in scoreArr:
        score1.append(entry[0])
        score2.append(entry[1])

    return [score1,score2]

def testDuctSizeRobustness():
    allScores = []
    widths=range(25,37)
    print widths
    for i in range(len(widths)):
        w = widths[i]
        h = widths[i]

        widthParams=["-W",str(w),"-H",str(h)]

        exArr = [ex]
        exArr.extend(widthParams)
        exArr.extend(exParams)
        exArr.extend(["-g","0"])

        print exArr
        subprocess.call(exArr)
        allScores.append(loadScores(fullRsrcDir))

        saveLoad("save","logs/ductwidth_scores.pickle",[allScores,widths])

    return

def testPosRobustness():
    allScores = []
    xs=np.arange(-5,5,0.5)
    zs=np.arange(-5,5,0.5)
    for i in range(len(xs)):
        x = xs[i]
        xscores = []
        for j in range(len(zs)):
            z = zs[j]
            posParams=["-x",str(x),"-z",str(z)]

            exArr = [ex]
            exArr.extend(posParams)
            exArr.extend(exParams)
            exArr.extend(["-g","0"])

            print posParams
            subprocess.call(exArr)
            xscores.append(loadScores(fullRsrcDir))

        allScores.append(xscores)

        mean = np.mean(allScores)
        std = np.std(allScores)

        saveLoad("save","logs/posscores.pickle",[mean,std,allScores,xs,zs])

    return

def plotDuctSizeResults():
    allScores = []
    widths=[]

    [allScores,widths]=saveLoad("load",pickleFile,[allScores,widths])

    widthvec=[]
    speedMeans=[]
    speedMaxes=[]
    speedStds=[]
    distMeans=[]
    distMaxes=[]
    distStds=[]
    for i in range(len(allScores)):
        widthvec.append(widths[i])
        temp=allScores[i]
        [speeds,dists]=getScores(temp)
        speedMeans.append(np.mean(speeds))
        speedMaxes.append(np.max(speeds))
        speedStds.append(np.std(speeds))
        distMeans.append(np.mean(dists))
        distMaxes.append(np.max(dists))
        distStds.append(np.std(dists))

    print np.mean(speedMaxes),np.std(speedMaxes)
    print np.mean(distMaxes),np.std(distMaxes)
    print len(speedMeans),len(speedStds),len(widthvec)
    print widthvec

    # Plot of Speed Means
    ind=np.arange(len(speedMeans))
    width=0.85

    fig, ax =plt.subplots()
    rects1 = ax.bar(ind,speedMeans,width,color='r',yerr=speedStds,label='Mean Speed (cm/s)')
    ax.set_xlabel('Duct Size (cm)')
    ax.set_ylabel('Performance')
    ax.set_title('Learned performance over various duct sizes')
    ax.set_xticks(ind+(width/2))
    ax.set_xticklabels(widthvec)
    plt.legend(loc='upper left')
    plt.show()

    # fig, ax =plt.subplots()
    # rects1 = ax.bar(ind,speedMeans,width,color='r',yerr=speedStds)
    # rects2 = ax.bar(ind+(width),np.array(distMeans)/100,width,color='y',yerr=np.array(distStds)/100,hatch='/')
    # ax.set_xlabel('Duct Size (cm)')
    # ax.set_ylabel('Performance')
    # ax.set_title('Learned performance over various duct sizes')
    # ax.set_xticks(ind+width)
    # ax.set_xticklabels(widthvec)
    # ax.legend((rects1[0], rects2[0]),('Speed Means (cm/s)', 'Distance/minute means (m)'), 'upper left')
    # plt.show()

    return

def plotPosResults():
    allScores = []
    xs=[]
    zs=[]
    mean=0
    std=0

    [mean,std,allScores,xs,zs]=saveLoad("load",pickleFile,[mean,std,allScores,xs,zs])

    xvec=[]
    zvec=[]
    speedMeans=[]
    speedMaxes=[]
    distMeans=[]
    distMaxes=[]
    for i in range(len(xs)):
        temp=allScores[i]
        for j in range(len(zs)):
            xvec.append(xs[i])
            zvec.append(zs[j])
            [speeds,dists]=getScores(temp[j])
            speedMeans.append(np.mean(speeds))
            speedMaxes.append(np.max(speeds))
            distMeans.append(np.mean(dists))
            distMaxes.append(np.max(dists))

    print np.mean(speedMaxes),np.std(speedMaxes)
    print np.mean(distMaxes),np.std(distMaxes)

    #Actual height histogram
    # fig=plt.figure()
    # ax=fig.add_subplot(111, projection='3d')
    # ax.scatter(xvec,zvec,distMaxes)
    # ax.set_xlabel('X position')
    # ax.set_ylabel('Z position')
    # ax.set_zlabel('Distance')
    # plt.show()

    # Plot of starting positions
    fig=plt.figure()
    plt.scatter(xvec,zvec)
    plt.xlabel('X position (cm)')
    plt.ylabel('Z position (cm)')
    plt.title('Starting Positions')
    plt.show()

    return

if __name__ == "__main__":
    args = parser.parse_args()

    rsrcDir=str(args.rsrcDir)
    fullRsrcDir=ntrtDir+"resources/src/"+rsrcDir+"/"
    ex=exDir+args.ex
    numEp=int(args.numEp)
    exParams=["-c","2","-d","--rot_y","1","-a","45","--duct_axis","1","-r",rsrcDir,"-s","60000","-e",str(numEp)]
    pickleFile=str(args.pfile)

    if args.method == 1:
        testPosRobustness()
    elif args.method == 2:
        testDuctSizeRobustness()
    elif args.method == 3:
        plotPosResults()
    elif args.method == 4:
        plotDuctSizeResults()
    # else:

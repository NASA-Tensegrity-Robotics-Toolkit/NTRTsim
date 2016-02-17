import copy
import random
import Algorithms

def elitism(elitismConfig,
            componentPopulation,
            scoreMethod="max",
            fitnessFunction="distance"):

    # selectionMethod = elitismConfig['selectionMethod']
    survivalCount = elitismConfig['survivalCount']
    localPopulation = copy.deepcopy(componentPopulation)

    if survivalCount > len(localPopulation):
        raise Exception("Cannot have more elites than there are components in a population.")

    outPopulation = []
    while len(outPopulation) < survivalCount:
        bestIndex = Algorithms.getBestComponentIndex(localPopulation, scoreMethod, fitnessFunction)
        newComponent = localPopulation.pop(bestIndex)
        outPopulation.append(newComponent)

    return outPopulation
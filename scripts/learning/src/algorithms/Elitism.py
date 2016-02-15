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

    outPopulation = []
    # TODO: Handle error here were survivalCount > len(previousPopulation)
    # localPopulation is automatically pruned each iteration with .pop
    while len(outPopulation) < survivalCount:
        bestIndex = Algorithms.getBestComponentIndex(localPopulation, scoreMethod, fitnessFunction)
        newComponent = localPopulation.pop(bestIndex)
        outPopulation.append(newComponent)

    return outPopulation
from helpersNew import dictTools
import random
import copy
import Algorithms

def crossOver(crossOverConfig,
              componentPopulation,
              rangeConfig,
              scoreMethod="max",
              fitnessFunction="distance"):

    # rangeDictionary is passed in to make sure that only elements
    # that are being learned over are used in crossOver

    outPopulation = []

    # selectionMethod = crossOverConfig['selectionMethod']
    survivalCount = crossOverConfig['spawnCount']
    localPopulation = copy.deepcopy(componentPopulation)

    for iteration in range(survivalCount):
        # Get first parent
        parentAIndex = Algorithms.getComponentIndexByProbability(localPopulation, scoreMethod, fitnessFunction)
        parentA = localPopulation[parentAIndex]

        # Remove parentA from parentB options
        remainingPopulation = localPopulation[:parentAIndex] + localPopulation[parentAIndex + 1:]

        # Get second parent
        parentBIndex = Algorithms.getComponentIndexByProbability(remainingPopulation, scoreMethod, fitnessFunction)
        parentB = remainingPopulation[parentBIndex]

        child = generateChild(parentA, parentB, rangeConfig)
        outPopulation.append(child)

        # print "parentA:"
        # print parentA
        # print "parentB:"
        # print parentB
        # print "child:"
        # print child
        # raw_input()

    # need to select part of the generation for gaussian sampling
    return outPopulation

# TODO: Make a proper testing infrastructure for crossover
# Current testing is very hacky: print parentA, parentB, child to terminal. copy to files. diff files.
# Still running ./testScript.sh to make sure that output format is the same as a known-good file

# TODO: childBase is likely mutated to child here
# taking output of mutateComponent for now just to make sure
def generateChild(parentA, parentB, rangeConfig):
    childBase = copy.deepcopy(parentA)
    configCount = Algorithms.getConfigCount(childBase, rangeConfig)
    mutateCount = random.randint(0, configCount)
    child, updatesPerformed = mutateComponent(childBase, parentB, rangeConfig, mutateCount)
    # Sanity checker. Can be removed once fully tested.
    if updatesPerformed != mutateCount:
        raise Exception("Child performed incorrect number of mutations:\n"
                        "mutateCount: " + str(mutateCount) + " updatedsPerformed: " + str(updatesPerformed))
    return child

#def generateComponentByRandom(parentA, parentB):
# Note that the order of parameters here is different than for the other algs
# Others are (varA, varB, rangeConfig)
# TODO: parentB is a terrible parameter name here
def mutateComponent(childBase, parentB, rangeConfig, updatesRemaining):
    dispatchMutateComponent = {
        type({}) : mutateDict,
        type([]) : mutateList,
        type('a') : mutateElement,
        type(1)   : mutateElement,
        type(0.1) : mutateElement,
        type(u'a'): mutateElement
    }
    return dispatchMutateComponent[type(childBase)](childBase, parentB, rangeConfig, updatesRemaining)

def mutateDict(childBase, parentB, rangeConfig, updatesRemaining):
    assert type(childBase) == type(rangeConfig) == type({}) == type(parentB)
    # TODO: Assumes that the key from rangeConfig is in baseDict and parentB. Should have exception handling.
    updatesPerformed = 0
    newDict = copy.deepcopy(childBase)
    if updatesRemaining < 0:
        raise Exception("Negative updates reamining in crossOver. this should never happen.")
    elif updatesRemaining > 0:
        for key in rangeConfig.keys():
            print key
            newValue, updatedCount = mutateComponent(childBase[key], parentB[key], rangeConfig[key], updatesRemaining)
            # print "before" + str(newDict[key])
            newDict[key] = newValue
            # print "after" + str(newDict[key])
            # dictTools.pause("comparing before-and-after newDict")
            updatesRemaining -= updatedCount
            updatesPerformed += updatedCount
            # TODO: Insert a break here if updatesRemaining == 0
    return (newDict, updatesPerformed)

def mutateElement(value, parentBValue, rangeConfig, updatesRemaining):
    # print "in mutateValue"
    #TODO: Allow different list-selection methods
    updatesPerformed = 0
    newValue = copy.deepcopy(value)
    # print "updatesRemaining:" + str(updatesRemaining)
    print "previousElement: " + str(value)
    print "newElement: " + str((parentBValue))
    if updatesRemaining > 0:
        if type(rangeConfig) == type([]):
            newValue = random.choice(rangeConfig)
        elif isMinMax(rangeConfig):
            newValue = parentBValue
        else:
            raise Exception("Recursed to value and rangeConfig is of unknown configuration:"
                            "\nValue: " + str(value) + "\nConfig: " + str(rangeConfig))
        updatesPerformed += 1
    # else:
    #     print "updatesRemaining == 0"
    return (newValue, updatesPerformed)

# Will this work for dicts in a list? Assuming yes, possible source of error
def mutateList(list, parentB, rangeConfig, updatesRemaining):
    updatesPerformed = 0
    newList = []
#    print "length before: " + str(len(list))
    for index in range(len(list)):
        element = list[index]
        # print "type before: " + str(type(element))
        newElement, updatedCount = mutateComponent(element, parentB[index], rangeConfig, updatesRemaining)
        # print "type after: " + str(type(newElement))
        if type(element) != type(newElement):
            print "newElement is of inconsistent type with origina!:"
            print "previousElement: " + str(element)
            print "newElement: " + str((newElement))
            dictTools.pause()
        newList.append(newElement)
        updatesRemaining -= updatedCount
        updatesPerformed += updatedCount
#    print "length after: " + str(len(newList))
#    dictTools.pause()
    return (newList, updatesPerformed)

def isMinMax(dictionary):
    dictKeys = dictionary.keys()
    dictKeys.sort()
    minMaxKeys = ["max", "min"]
    return dictKeys == minMaxKeys

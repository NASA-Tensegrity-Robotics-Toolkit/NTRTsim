import copy
import random
import Algorithms

# RangeConfig is needed because sampling may be over a discrete space
# i.e. with class types
def gaussianSampling(gaussianConfig,
                     rangeConfig,
                     componentPopulation):

    outPopulation = []
    for component in componentPopulation:
        newComponent = copy.deepcopy(component)
        newComponent = mutateComponent(newComponent, rangeConfig, gaussianConfig)
        outPopulation.append(newComponent)
    return outPopulation

# GenerateX assumes that rangeConfig is a dictionary except for generateValue

# This is a complex way of updating a component based on configs
# It uses a dispatcher in anticipation of the complexities of structure learning
# In addition, this will better support learning from structure files with substructures

# This also addresses nested-list formats used in previous versions of NTRT
# It also makes no assumptions about the structure of the passed-in component
# The only assumptions are that rangeConfig terminates in a minMaxDict or list
# and gaussainConfig being a dictionary with "mutationDev" and "mutationChance".
# Theoretically, gaussianConfig could also be element-based

def mutateComponent(component, rangeConfig, gaussianConfig):
    dispatchMutateComponent = {
        type({}) : mutateDict,
        type([]) : mutateList,
        type('a') : mutateElement,
        type(1)   : mutateElement,
        type(0.1) : mutateElement,
        type(u'a'): mutateElement
    }
    return dispatchMutateComponent[type(component)](component, rangeConfig, gaussianConfig)

def mutateDict(dictionary, rangeConfig, gaussianConfig):
    assert type(dictionary) == type(rangeConfig) == type({})
    newDict = copy.deepcopy(dictionary)
    for key in rangeConfig.keys():
        newDict[key] = mutateComponent(newDict[key], rangeConfig[key], gaussianConfig)
    return newDict

def mutateElement(value, rangeConfig, gaussianConfig):
    # print "in mutateValue"
    newValue = value
    if random.random() < gaussianConfig["mutationChance"]:
        # print "mutationChance successful"

        #TODO: Allow different list-selection methods
        if type(rangeConfig) == type([]):
            newValue = random.choice(rangeConfig)
        elif isMinMax(rangeConfig):
            # print "rangeConfig isMinMax==true."
            # calculation an adjustment value based on the range in the config
            # NOTE!!! This is slightly different from Brian's implementation (see Readme)
            deviation = random.uniform(0, gaussianConfig["mutationDev"])
            scale = (rangeConfig["max"] - rangeConfig["min"]) / 2.0
            average = (rangeConfig["max"] + rangeConfig["min"]) / 2.0
            adjustment = deviation * scale * average
            newValue += adjustment

            # TODO: The logic is sound, but this smells like a learning inefficiency
            # When a component is at a limit, its deviations can only go in one direction
            # Thus, it is only mutating 1/2 as much
            # Further, it must mutate sufficienctly to "escape" the limit, otherwise it will just go back.
            if newValue > rangeConfig["max"]:
                newValue = rangeConfig["max"]
            elif newValue < rangeConfig["min"]:
                newValue = rangeConfig["min"]

            # print "rangeConfigMax: " + str(rangeConfig["max"])
            # print "rangeConfigMin: " + str(rangeConfig["min"])
            # print "deviation: " + str(deviation)
            # print "scale: " + str(scale)
            # print "adjustment: " + str(adjustment)
            # print "value: " + str(value)
            # print "newValue: " + str(newValue)
        else:
            raise Exception("Recursed to value and rangeConfig is of unknown configuration:"
                            "\nValue: " + str(value) + "\nConfig: " + str(rangeConfig))
    # print "value: " + str(value)
    # print "newValue: " + str(newValue)
    return newValue

def mutateList(list, rangeConfig, gaussianConfig):
    # print "in mutateList"
    # print str(list)
    newList = []
    for element in list:
        # print "element: " + str(element)
        newElement = mutateComponent(element, rangeConfig, gaussianConfig)
        # print "newElement: " + str(newElement)
        newList.append(newElement)
    return newList

def isMinMax(dictionary):
    dictKeys = dictionary.keys()
    dictKeys.sort()
    minMaxKeys = ["max", "min"]
    return dictKeys == minMaxKeys

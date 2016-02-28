import Algorithms
import random
import copy
from helpersNew import Generation
from helpersNew import dictTools

def monteCarlo(monteCarloConfig, rangeConfig, templateComponent):

    newComponentPopulation = []
    # Not sure how dictionary passing/copying is working out here. check it later.
    for item in range(monteCarloConfig['spawnCount']):
        # newComponent = generateNewComponent(rangeConfig, copy.deepcopy(templateComponent))
        newComponent = generateComponent(copy.deepcopy(templateComponent), rangeConfig)
        # print newComponent
        # dictTools.pause()
        newComponentPopulation.append(newComponent)
    return newComponentPopulation

def generateComponent(component, rangeConfig):
    dispatchMutateComponent = {
        type({}) : generateDict,
        type([]) : generateList,
        type('a') : generateElement,
        type(1)   : generateElement,
        type(0.1) : generateElement,
        type(u'a'): generateElement
    }
    # print "Component:~~~~~~"
    # print component
    # print "rangeConfig:~~~~~~~"
    # print rangeConfig
    # dictTools.pause()
    return dispatchMutateComponent[type(component)](component, rangeConfig)

def generateDict(dictionary, rangeConfig):
    assert type(dictionary) == type(rangeConfig) == type({})
    newDict = copy.deepcopy(dictionary)
    for key in rangeConfig.keys():
        newDict[key] = generateComponent(newDict[key], rangeConfig[key])
    return newDict

def generateElement(value, rangeConfig):
    #TODO: Allow different list-selection methods
    if type(rangeConfig) == type([]):
        newValue = random.choice(rangeConfig)
    elif Algorithms.isMinMax(rangeConfig):
        newValue = random.uniform(rangeConfig["min"], rangeConfig["max"])
    else:
        raise Exception("Recursed to value and rangeConfig is of unknown configuration:"
                        "\nValue: " + str(value) + "\nConfig: " + str(rangeConfig))
    return newValue

def generateList(list, rangeConfig):
    # print "in mutateList"
    # print str(list)
    newList = []
    for element in list:
        # print "element: " + str(element)
        newElement = generateComponent(element, rangeConfig)
        # print "newElement: " + str(newElement)
        newList.append(newElement)
    return newList
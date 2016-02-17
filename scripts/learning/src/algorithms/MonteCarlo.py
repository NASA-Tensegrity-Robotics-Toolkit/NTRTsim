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

# TODO: Refactor this to follow the dispatcher pattern in dictTools for compareDeepType
def generateNewComponent(rangeDictionary, component, tagStack=None):
    if not tagStack:
        tagStack = []
    for key, value in component.iteritems():
        nextTagStack = list(tagStack)
        nextTagStack.append(key)
        if type(value) == type({}):
            component[key] = generateNewComponent(rangeDictionary, value, nextTagStack)
        elif type(value) == type([]):
            newList = generateNewComponentFromList(rangeDictionary, value, nextTagStack)
            """
            TODO:
            There is a case here where if there are some values already set in a nested list,
            they will be overwritten with null.
            Do a "deep sweep" to make sure that this doesn't happen.
            """
            component[key] = newList
        # Assume that value is an integer
        else:
            newItem = getValueFromTagStack(rangeDictionary, nextTagStack)
            # Check that we actually updated the value
            if newItem:
                component[key] = newItem
    return component

def generateNewComponentFromList(rangeDictionary, value, tagStack):
    if type(value) == type([]):
        newValue = []
        for item in value:
            # May not need a new list(tagStack) here. doing it as a precaution
            newItem = generateNewComponentFromList(rangeDictionary, item, list(tagStack))
            newValue.append(newItem)
    else:
        newValue = getValueFromTagStack(rangeDictionary, list(tagStack))

    return newValue

def getValueFromTagStack(rangeDictionary, tagStack):
    newItem = None
    baseTag = tagStack[-1]
    while not newItem:
        if len(tagStack) > 0:
            currentTag = tagStack.pop()
            #print "currentTag: " + currentTag
        else:
            # We didn't find a tag for it.
            # See block comment above
            break
        # THIS IS CASE SENSITIVE FOR NOW
        if currentTag in rangeDictionary:
            #print "Found currentTag in dictionary: " + currentTag
            if type(rangeDictionary[currentTag]) == type([]):
                newItem = random.choice(rangeDictionary[currentTag])
            elif type(rangeDictionary[currentTag]) == type({}):
                try:
                    min = rangeDictionary[currentTag]['min']
                    max = rangeDictionary[currentTag]['max']
                    newItem = random.uniform(min, max)
                except KeyError:
                    print rangeDictionary[currentTag]
                    print "tagStack: " + str(tagStack)
                    print "currentTag: " + str(currentTag)
                    raise Exception("Couldn't find min/max range values for tag " + currentTag + ". Check your range section of the learning spec.")
            else:
                raise Exception("rangeDictionary terminates in neither an options list or minMax.")
            #if not newItem:
        #print "Could not find a match for base tag: " + baseTag
    return newItem
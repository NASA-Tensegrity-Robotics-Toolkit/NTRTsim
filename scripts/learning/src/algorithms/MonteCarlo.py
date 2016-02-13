import random
from helpersNew import Generation
from helpersNew import dictTools

class MonteCarlo:

    """
    Performs basic Monte Carlo by replicating the given component.
    All of the attributes of the component are randomized based on the config.

    """

def monteCarlo(monteCarloConfig, rangeDictionary, templateComponent):

    newComponentPopulation = []
    # Not sure how dictionary passing/copying is working out here. check it later.
    # spawned = 0
    for item in range(monteCarloConfig['spawnCount']):
        newComponent = generateNewComponent(rangeDictionary, templateComponent.copy())
        newComponentPopulation.append(newComponent)
        # spawned += 1
    return newComponentPopulation

def generateNewComponent(rangeDictionary, componentDictionary, tagStack=None):
    if not tagStack:
        tagStack = []
    for key, value in componentDictionary.iteritems():
        nextTagStack = list(tagStack)
        nextTagStack.append(key)
        if type(value) == type({}):
            componentDictionary[key] = generateNewComponent(rangeDictionary, value, nextTagStack)
        elif type(value) == type([]):
            newList = generateNewComponentFromList(rangeDictionary, value, nextTagStack)
            """
            TODO:
            There is a case here where if there are some values already set in a nested list,
            they will be overwritten with null.
            Do a "deep sweep" to make sure that this doesn't happen.
            """
            componentDictionary[key] = newList
        # Assume that value is an integer
        else:
            newItem = getValueFromTagStack(rangeDictionary, nextTagStack)
            # Check that we actually updated the value
            if newItem:
                componentDictionary[key] = newItem
    return componentDictionary

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


"""
Walking through a rangedictionary/tag stack SHOULD NOT BE NECESSARY!!!
The rangeDictionary must include all the parameters&ranges you want to learn over,
in a 1-D dictionary.
This means that the tag you are looking up just has to do a direct check in the
range dictionary, nothing more.

TODO:
Allow the "getValueFromTagStack" to evaluate more than just a min/max range.
Allow it to also do an enum (check if type is list, if so then enum, else range.)

***Consider:
Non-float ranges? integers only?
"""

"""
It would be nice to have running out of stack being an error flag, but that won't work.
In the event that a learning run is done where not all of the parameters are learned
(i.e. builder only learns on a specific set of tags)
then this would throw errors and not complete.
Instead, we'll throw warnings WHEN LOGGING WORKS and let the user proceed.
"""
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
            try:
                min = rangeDictionary[currentTag]['min']
                max = rangeDictionary[currentTag]['max']
            except KeyError:
                raise Exception("Couldn't find min/max range values for tag " + currentTag + ". Check your range section of the learning spec.")
            newItem = random.uniform(min, max)
    #if not newItem:
        #print "Could not find a match for base tag: " + baseTag
    return newItem
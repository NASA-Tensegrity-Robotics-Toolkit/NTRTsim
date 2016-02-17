import copy
import random
from MonteCarlo import monteCarlo
from Elitism import elitism
from GaussianSampling import gaussianSampling
from CrossOver import crossOver


def dispatchLearning(componentConfig,
                     scoreMethod="max",
                     fitnessFunction="distance",
                     componentPopulation=None):

    algorithms = componentConfig['Algorithms']
    """
    Cannot rely on yaml to return an in-order dictionary.
    Thus, the order is instead inforced here

    """
    # TODO: validity of component values after mutation (vs. rangeDictionary)
    newComponentPopulation = copy.deepcopy(componentPopulation)
    if "MonteCarlo" in algorithms:
        print "in MonteCarlo"
        templateComponent = getBestComponent(componentPopulation, scoreMethod, fitnessFunction)
        newComponentPopulation = monteCarlo(monteCarloConfig=algorithms['MonteCarlo'],
                                            rangeConfig=componentConfig['Ranges'],
                                            templateComponent=templateComponent)
        # MonteCarlo is a stand-alone algorithm.
        # The other algorithms can be used in concert.
        # Theoretically, this doesn't need to be here.

    if "Elitism" in algorithms:
        print "in Elitism"
        #print "preElitism popSize: " + str(len(newComponentPopulation))
        newComponentPopulation = elitism(elitismConfig=algorithms['Elitism'],
                                         componentPopulation=newComponentPopulation,
                                         scoreMethod=scoreMethod,
                                         fitnessFunction=fitnessFunction)
        #print "postElitism popSize: " + str(len(newComponentPopulation))
        #print newComponentPopulation[0]
        #raw_input("check this component from Elitism.")

    if "GaussianSampling" in algorithms:
        # print "in GaussianSampling"
        # print "before:"
        # print newComponentPopulation[0]
        newComponentPopulation += gaussianSampling(gaussianConfig=algorithms['GaussianSampling'],
                                                  rangeConfig=componentConfig['Ranges'],
                                                  componentPopulation=newComponentPopulation)
        # print "after:"
        # print newComponentPopulation[0]
        # raw_input("check this component from Gaussian Sampling.")

    if "CrossOver" in algorithms:
        print "in CrossOver"
        newComponentPopulation += crossOver(crossOverConfig=algorithms['CrossOver'],
                                           rangeConfig=componentConfig['Ranges'],
                                           componentPopulation=newComponentPopulation,
                                           scoreMethod=scoreMethod,
                                           fitnessFunction=fitnessFunction)

    populationID = 0
    for component in newComponentPopulation:
        component['populationID'] = populationID
        populationID += 1

    # Map componentMember to each of the possible algorithms
    return newComponentPopulation

# Once components are modeled as ComponentPopulation and Component objects, this section will move to Generationmanager
# See the ReadMe

def getComponentByRandom(componentPopulation):
    randomComponent = random.choice(componentPopulation)
    return copy.deepcopy(randomComponent)

def getBestComponent(componentPopulation, scoreMethod="max", fitnessFunction="distance"):
    index = getBestComponentIndex(componentPopulation, scoreMethod, fitnessFunction)
    return componentPopulation[index]

def getBestComponentIndex(componentPopulation, scoreMethod="max", fitnessFunction="distance"):
    maxScore = getComponentScore(componentPopulation[0], scoreMethod, fitnessFunction)
    maxIndex = 0
    # If any don't have scores, you will get an error
    for index in range(0, len(componentPopulation)):
        component = componentPopulation[index]
        componentScore = getComponentScore(component, scoreMethod, fitnessFunction)
        if componentScore > maxScore:
            maxScore = componentScore
            maxIndex = index
        index += 1

    return maxIndex

# Unused
# Not deleting -- will be moved to ComponentPopulation object later
def getComponentByProbability(componentPopulation, scoreMethod="max", fitnessFunction="distance"):
    selectIndex = getComponentIndexByProbability(componentPopulation, scoreMethod, fitnessFunction)
    chosenComponent = componentPopulation[selectIndex]
    return copy.deepcopy(chosenComponent)

# TODO: Check the logic of this
def getComponentIndexByProbability(componentPopulation, scoreMethod="max", fitnessFunction="distance"):
    totalScore = getNormalizedTotalScore(componentPopulation, scoreMethod, fitnessFunction)
    targetScore = random.uniform(0, totalScore)
    minScore = getMinScore(componentPopulation, scoreMethod, fitnessFunction)
    currentScore = 0
    selectIndex = None
    for index in range(len(componentPopulation)):
        component = componentPopulation[index]
        # Adjust for negative scores
        currentScore -= minScore
        currentScore += getComponentScore(component, scoreMethod, fitnessFunction)
        if currentScore > targetScore:
            selectIndex = index
            break

    return selectIndex

# Fitness scores can have negative values
# To adjust for this, gross each score up by the lowest score
def getNormalizedTotalScore(componentPopulation, scoreMethod="max", fitnessFunction="distance"):
    totalScore = 0
    minScore = getComponentScore(componentPopulation[0], scoreMethod, fitnessFunction)
    # If any don't have scores, you will get an error
    for component in componentPopulation:
        componentScore = getComponentScore(component, scoreMethod, fitnessFunction)
        totalScore += componentScore
        if componentScore < minScore:
            minScore = componentScore

    # Gross up scores for negative values
    totalScore += minScore * len(componentPopulation)

    return totalScore

def getMinScore(componentPopulation, scoreMethod="max", fitnessFunction="distance"):
    minScore = getComponentScore(componentPopulation[0], scoreMethod, fitnessFunction)
    # If any don't have scores, you will get an error
    for component in componentPopulation:
        componentScore = getComponentScore(component, scoreMethod, fitnessFunction)
        if componentScore < minScore:
            minScore = componentScore

    return minScore

def getComponentScore(component, scoreMethod="max", fitnessFunction="distance"):

    # Backwards compatibility
    # Scores in components don't indicate source
    # No scores yet assigned to this component.
    # Should never get to Elitism if this is the case.
    # TODO: THIS IS A HACK
    if "scores" not in component or len(component["scores"]) == 0:
        # Bad form to return mid-function
        return -1000
    elif type(component["scores"][0]) == type(1):
        fitnessScores = component["scores"]
    elif type(component["scores"][0]) == type({}):
        fitnessScores = [score[fitnessFunction] for score in component["scores"]]
    else:
        raise Exception("Could not read scores from component.")
    dispatchScoring = {
        "max" : computeComponentMaxScore,
        "average" : computeComponentAverageScore,
    }
    return dispatchScoring[scoreMethod](fitnessScores)

def computeComponentMaxScore(scores):
    if len(scores) == 0:
        maxScore = None
        # This may through an error later
        # TODO: Is this the best way to handle empty scores?
    else:
        maxScore = scores[0]
        for score in scores:
            if score > maxScore:
                maxScore = score
    return maxScore

def computeComponentAverageScore(scores):
    if len(scores) == 0:
        avgScore = None
        # This may through an error later
        # TODO: Is this the best way to handle empty scores?
    else:
        avgScore = sum(scores) / len(scores)
    return avgScore

# Ranges format:
# tag:
#     tag:
#         min: number
#         max: number
#     tag:
#         - option1
#         - option2

# Get the number of rangeConfig items in dict
# This can be generalized
# Could have a list of iterable elements, hence recursing on each one
# See ReadMe.md about assumptions for Ranges
# TODO: Rename. This is closer to config = subset of element, get the # of shared elements
def getConfigCount(element, keyConfig):
    count = 0
    if type(element) == type({}):
        for key in keyConfig.keys():
            # print "key: " + str(key)
            # print "inKeyConfig: " + str(key in keyConfig.keys())
            # print str(element.keys())
            count += getConfigCount(element[key], keyConfig[key])
    elif type(element) == type([]):
        for entry in element:
            count += getConfigCount(entry, keyConfig)
    # If element is a tuple, this is inaccurate
    # Make sure that config is at a terminating type
    elif type(keyConfig) == type([]) or isMinMax(keyConfig):
        count += 1
    else:
        raise Exception("Unknown element/config combination in getConfigCount")
    return count

def isMinMax(dictionary):
    dictKeys = dictionary.keys()
    dictKeys.sort()
    minMaxKeys = ["max", "min"]
    return dictKeys == minMaxKeys

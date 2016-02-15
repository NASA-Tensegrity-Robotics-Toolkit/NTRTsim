import copy
import random
from MonteCarlo import monteCarlo
from Elitism import elitism
from GaussianSampling import gaussianSampling


def dispatchLearning(componentConfig,
                     scoreMethod="max",
                     fitnessFunction="distance",
                     baseComponent=None,
                     componentPopulation=None):

    """
    Cannot rely on yaml to return an in-order dictionary.
    Thus, the order is instead inforced here

    """
    # TODO: validity of component values after mutation (vs. rangeDictionary)
    newComponentPopulation = copy.deepcopy(componentPopulation)
    if "MonteCarlo" in componentConfig['Algorithms']:
        newComponentPopulation = monteCarlo(monteCarloConfig=componentConfig['Algorithms']['MonteCarlo'],
                                            rangeConfig=componentConfig['Ranges'],
                                            templateComponent=baseComponent)
        # MonteCarlo is a stand-alone algorithm.
        # The other algorithms can be used in concert.
        # Theoretically, this doesn't need to be here.

    if "Elitism" in componentConfig['Algorithms']:
            newComponentPopulation = elitism(elitismConfig=componentConfig['Algorithms']['Elitism'],
                                             componentPopulation=newComponentPopulation,
                                             scoreMethod=scoreMethod,
                                             fitnessFunction=fitnessFunction)
            #print newComponentPopulation[0]
            #raw_input("check this component from Elitism.")

    if "GaussainSampling" in componentConfig['Algorithms']:
            # print "before:"
            # print newComponentPopulation[0]
            newComponentPopulation = gaussianSampling(gaussianConfig=componentConfig['Algorithms']['GaussianSampling'],
                                                      rangeConfig=componentConfig['Ranges'],
                                                      componentPopulation=newComponentPopulation)
            # print "after:"
            # print newComponentPopulation[0]
            # raw_input("check this component from Gaussian Sampling.")

        # elif algorithm == "CrossOver":
        #     return
        # else:
        #     raise Exception("Unknown algorithm \"" + algorithm + "\" passed in. Check the Algorithms sections of your yaml learning spec.")
    populationID = 0
    for component in newComponentPopulation:
        component['populationID'] = populationID
        populationID += 1

    # Map componentMember to each of the possible algorithms
    return newComponentPopulation

def getComponentByRandom(componentPopulation):
    randomComponent = random.choice(componentPopulation)
    return copy.deepcopy(randomComponent)

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

# TODO: Check the logic of this
def getComponentByProbability(componentPopulation, scoreMethod="max", fitnessFunction="distance"):
    totalScore = getNormalizedTotalScore(componentPopulation, scoreMethod, fitnessFunction)
    targetScore = random.uniform(0, totalScore)
    minScore = getMinScore(componentPopulation, scoreMethod, fitnessFunction)
    currentScore = 0
    chosenComponent = None
    for component in componentPopulation:
        # Adjust for negative scores
        currentScore -= minScore
        currentScore += getComponentScore(component, scoreMethod, fitnessFunction)
        if currentScore > targetScore:
            chosenComponent = component
            break

    return copy.deepcopy(chosenComponent)

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
    if len(component["scores"]) == 0:
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

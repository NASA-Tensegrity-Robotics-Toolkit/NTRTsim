


def dispatchLearning(componentName=None,
                     componentConfig=None,
                     baseComponent=None,
                     componentPopulation=None):

    """
    Cannot rely on yaml to return an in-order dictionary.
    enforce the order here by instead doing
    if montecarlo in -> montecarlo
    if elitism in -> ... etc
    """
    newComponentPopulation = []
    for algorithm in componentConfig['Algorithms']:
        print algorithm
        if algorithm == "MonteCarlo":
            newComponentPopulation = monteCarlo(componentConfig['Algorithms']['MonteCarlo'], componentConfig['Ranges'], baseComponent)
        """
        elif algorithm == "Elitism":
            return
        elif algorithm == "GaussianSampling":
            return
        elif algorithm == "CrossOver":
            return
        else:
            raise Exception("Unknown algorithm \"" + algorithm + "\" passed in. Check the Algorithms sections of your yaml learning spec.")
        """
    populationID = 0
    for component in newComponentPopulation:
        component['populationID'] = populationID
        populationID += 1

    # Map component to each of the possible algorithms
    return newComponentPopulation

from MonteCarlo import monteCarlo

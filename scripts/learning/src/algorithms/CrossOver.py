import random
from GenerationManager import Generation

class CrossOver:
    """
    Performs basic genetic algorithm on the given generation.
    
    Configuration Format:
    Number of Offspring = integer
    Selection Method = "random" or "50/50"

    YAML Format:
    CrossOver:
        spawnCount: int
        selectionMethod: "50/50" or "random"
        mutationChance: float

    Assumes that the sum of scores is expressible as an integer.
    """
    
    def generateOffspring(self, learningConfig, generation, component="All"):
        """
        learningConfig -- The dictionary parsed by LearningJobMaster with specifications
                          for how to perform the CrossOver algorithm.
        parameterSubset (optional) -- The string indexing the dictionary to a parameter array
                                      if absent, assumed to be entire set of parameters.
        generation -- A LearningDictionary containing members with scores.
        """

        self._offspringCount = learningConfig['spawnCount']
        self._selectionMethod = learningConfig['selectionMethod']
        self._component = component
        self._generation = generation

        return self._createOffspring(self, generation)

    def _createOffspring(self, generation):
        totalScore = generation.getTotalScore()

        newComponents = []

        for i in range(self._offspringCount):

            parents = self._getParents(self._generation)
            child = self._createChild(self, parents[0], parents[1])
            newComponents.append(child)
        
        return newComponents
    
    # Should this take a tuple or an array instead of two parents?
    def _createChild(self, parentA, parentB):
        # Assumed that the two parents are of the same type, length, etc...
        # This DOES NOT support a nested structure as you'd find with parameters
        # Will work for controllers where a component just has a list of floats
        parentAValues = list(parentA.components[self._component])
        parentBValues = list(parentB.components[self._component])

        splitIndex = random.random() * len(parentAValues)

        childValues = parentAValues[:splitIndex]+parentBValues[splitIndex:]
        child = {
            self._component : childValues
            }
        return child

    def _getParents(self, generation):
        totalScore = generation.getTotalScore()
        parentAIndex = random.randint(0, totalScore)
        parentBIndex = random.randint(0, totalScore)

        sum = 0
        parentA = None
        parentB = None

        for index in len(generation):
            member = generation[index]
            sum += member.getScore()
            index += 1
            if sum > parentAIndex and not parentA:
                parentA = member
            if sum > parentBIndex and not parentB:
                parentB = member
            # == may need to be replaced by 'is' here
            # Parents cannot be the same.
            # Randomly select a second new parent
            if parentA == parentB:
                parentBIndex = random.randint(0, totalScore)
                index = 0
            # Found two valid parents, can exit
            elif parentA and parentB:
                break
        # <TODO> Catch case of no 2 parents found

        return (parentA, parentB)
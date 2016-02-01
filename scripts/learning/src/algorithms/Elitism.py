import random
from helpersNew import Generation

class Elitism:
    
    def elitism(self, learningConfig, generation):
        
        method = learningConfig['selectionMethod']
        count = learningConfig['survivalCount']
        outGeneration = []
        
        # Copy to avoid accidentally mutating the generation argument
        generationLocal = list(generation)

        if count > len(generation):
            raise Exception("Elitism trying to keep more members than there are in the generation.")
        elif method == "absolute":
            generationLocal.sortMembers()
            outGeneration = generationLocal[:count]
        elif method == "probability":
            for index in range(count):
                newMember = self._popByProbability(generationLocal)
                outGeneration.append(newMember)
        else:
            raise Exception("Did not recognize selection method " + method + " in Elitism.")
        
        return outGeneration

    def _popByProbability(self, generation):
        """
        TODO
        THIS CODE IS NEARLY COPY-PASTED FROM CROSSOVER
        Either the learning methods should fall under one class
        or this should be generalized and imported.
        """
        totalScore = generation.getTotalScore()
        selectScore = random.randint(0, totalScore)

        sum = 0
        popIndex = None
        for index in range(len(generation)):
            member = generation[index]
            sum += member.getScore()
            index += 1
            if sum > selectScore:
                popIndex = index
                break

        if not popIndex:
            raise Exception("Could not find a member for selection in Elitism.")

        return generation.pop(popIndex)
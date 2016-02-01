import random
from GenerationManager import Generation

class NeuralNetwork:


    def generateOffspring(self, learningConfig, generation, component="All"):





    def dummy():
        # Neural network or not. Slightly different data structure
        if params['numberOfStates'] == 0 :

            newParams = []

            for i in range(0, params['numberOfInstances']) :
                subParams = []
                for j in range(0, params['numberOfOutputs']) :
                    # Assume scaling happens elsewhere
                    subParams.append(random.uniform(pMin, pMax))
                newParams.append(subParams)
        else :
            newParams = { 'numActions' : params['numberOfOutputs'],
                         'numStates' : params['numberOfStates'],
                         'numHidden' : params['numberHidden'],
                         'neuralFilename' : "logs/bestParameters-test_fb-"+ newController['paramID'] +".nnw"}

            neuralParams = []

            numStates = params['numberOfStates']
            numOutputs=  params['numberOfOutputs']
            numHidden = params['numberHidden']

            totalParams = (numStates + 1) * (numHidden) + (numHidden + 1) * numOutputs

            for i in range(0,  totalParams) :
                neuralParams.append(random.uniform(pMin, pMax))

            newParams['neuralParams'] = neuralParams

            self.__writeToNNW(neuralParams, self.path + newParams['neuralFilename'])

        newController['params'] = newParams
        newController['scores'] = []

        return newController


    # Dummy function
    # DO NOT CALL
    def neuro(self):
        params={}
        cNew={}
        if(params['numberOfStates'] > 0):
            newNeuro = {}
            newNeuro['neuralParams'] = cNew
            newNeuro['numStates'] = params['numberOfStates']
            newNeuro['numActions'] =  params['numberOfOutputs']
            newNeuro['numHidden'] =  params['numberHidden']
            return newNeuro
        else:
            return cNew






    def __writeToNNW(self, neuralParams, fileName):
        """
        Take the params (neuralParams) and write them to a .nnw file for reading by
        the neuralNetwork code (Third party library)
        """
        fout = open(fileName, 'w')
        first = True
        for x in neuralParams:
            if (first):
                fout.write(str(x))
                first = False
            else:
                fout.write("," + str(x))




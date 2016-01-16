    def __getNewParams(self, paramName):
        """
        Generate a new set of paramters based on learning method and config file
        Returns a dictionary paramName : values
        This is the implementation of monteCarlo
        """
        params = self.jConf["learningParams"][paramName]

        pMax = params['paramMax']
        pMin = params['paramMin']

        newController = {}
        newController['paramID'] = str(self.paramID)

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
    def __mutateParams(self, currentController, paramName):
        """
        Change some parameters (according to mutationChance) by an amount sampled from a gaussian
        (mutationDev)
        """
        params = self.jConf["learningParams"][paramName]

        pMax = params['paramMax']
        pMin = params['paramMin']

        # Slightly different encoding for neural nets
        if(params['numberOfStates'] > 0):
            cNew = list(currentController['neuralParams'])
        else:
            # Make a deep copy
            cNew = list(currentController)

        for i, v in enumerate(cNew):
            if isinstance(v, collections.Iterable):
                v = self.__mutateParams(v, paramName)
            else:
                if(random.random() > params['mutationChance']):

                    mutAmount = random.normalvariate(0.0, params['mutationDev'])
                    v += mutAmount

                    if (v > pMax):
                        v = pMax
                    elif (v < pMin):
                        v = pMin
            cNew[i] = v

        # Slightly different encoding for neural nets
        if(params['numberOfStates'] > 0):
            newNeuro = {}
            newNeuro['neuralParams'] = cNew
            newNeuro['numStates'] = params['numberOfStates']
            newNeuro['numActions'] =  params['numberOfOutputs']
            newNeuro['numHidden'] =  params['numberHidden']
            return newNeuro
        else:
            return cNew
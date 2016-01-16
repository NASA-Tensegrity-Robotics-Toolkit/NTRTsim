    def __getChildController(self, c1, c2, params):
        """
        Takes two controllers and merges them with a 50/50 chance of selecting
        a parameter from each controller
        """

        if(params['numberOfStates'] > 0):
            c1 = list(c1['neuralParams'])
            c2 = list(c2['neuralParams'])

        cNew = []

        if (len(c1) == 0):
            raise NTRTMasterError("Error in length")

        """
        # Old code for random params
        for i, j in zip(c1, c2):
            # Go to the deepest level of the parameters (see instances in the specification)
            if isinstance(i, collections.Iterable):
                cNew.append(self.__getChildController(i, j, params))
            else:
                # @todo should this be adjustable?
                if (random.random() > 0.5):
                    cNew.append(i)
                else:
                    cNew.append(j)
        """
        crossOver = random.randint(0, len(c1) - 1)

        print(crossOver)

        cNew[0:crossOver] = c1[0:crossOver]
        print(len(cNew))

        cNew[crossOver:len(c2)+1] = c2[crossOver:len(c2)+1]
        print(len(cNew))


        if (len(cNew) != len(c1)):
            raise NTRTMasterError("Error in length")

        if(params['numberOfStates'] > 0):
            newNeuro = {}
            newNeuro['neuralParams'] = cNew
            newNeuro['numStates'] = params['numberOfStates']
            newNeuro['numActions'] =  params['numberOfOutputs']
            newNeuro['numHidden'] =  params['numberHidden']
            return newNeuro
        else:
            return cNew


    def __getControllerFromProbability(self, currentGeneration, prob):
        """
        A support function for genetic algorithms that selects a controller
        based on the distribution of probabilities for all of their controllers (their
        contriution to the total score of the generation)
        """
        for c in currentGeneration.itervalues():
            if (c['probability'] < prob):
                break

        return c

        raise NTRTMasterError("Insufficient values to satisfy requested probability")



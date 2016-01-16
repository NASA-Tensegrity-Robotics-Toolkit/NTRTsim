# TODO check that both of these imports are needed
from collections import OrderedDict
import collections

class LearningDictionary(collections.OrderedDict):
    """
    Dictionary for storing entities for learning trials.
    Dynamically calculates all accessible properties.
    """

    def __init__(self, ID):
        """
        Constructor for a LearningDictionary.
        Requires an ID to reference the dictionary.
        """
        # Create internal variables for dynamic calculation
        self.ID = ID
        self._maxScoreUpdated = False
        self._maxScore = 0
        self._minScoreUpdated = False
        self._minScore = 0
        self._avgScoreUpdated = False
        self._avgScore = 0

        collections.OrderedDict.__init__(self)
    
    def __setitem__(self, key, value):
        """
        If the domain element already appears in the dictionary, delete it.
        Add the given pair as the last element in the dictionary.
        Calculates dynamically.
        
        key -- Another LearningDictionary.
        value -- Fitness score of the LearningDictionary's elements.
        """
        if key in self:
            del self[key]
        # Although the same domain element is added, the range element could be different.
        # Thus, the average and max scores could change.
        self._needsUpdate()
        collections.OrderedDict.__setitem__(self, key, value)

    def _needsUpdate(self):
        """
        Flags all calculated values as requiring updating.
        """
        self._maxScoreUpdated = False
        self._avgScoreUpdated = False
        self._minScoreUpdated = False

    def getAvgScore(self):
        """
        Returns the average score of the elements in the dictionary.
        Calculates dynamically.
        """
        sum   = 0
        count = 0
        if not self._avgScoreUpdated:
            for value in collections.OrderedDict.itervalues(self):
                sum   += value
                count += 1    
            self._avgScore = sum / count
            self._avgScoreUpdated = True
        return self._avgScore

    def getMaxScore(self):
        """
        Returns the maximum score of the elements in the dictionary.
        Calculates dynamically.
        """
        if not self._maxScoreUpdated:
            for value in collections.OrderedDict.itervalues(self):
                self._maxScore = max(self._maxScore, value)
            self._maxScoreUpdated = True
        return self._maxScore

    def getMinScore(self):
        """
        Returns the minimum of the value elements in the dictionary.
        Calculates dynamically.
        """
        if not self._minScoreUpdated:
            for value in collections.OrderedDict.itervalues(self):
                self._minScore = min(self._minScore, value)
            self._minScoreUpdated = True
        return self._minScore

### Implementing code, not for final commit
from random import randint
x = LearningDictionary(1)
for i in range(0, 10):
    x[i] = randint(0, 11)
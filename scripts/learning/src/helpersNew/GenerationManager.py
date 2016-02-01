# TODO check that both of these imports are needed
from collections import OrderedDict
import collections
from helpersNew import dictTools

#
# THIS WILL LIKELY BE CHANGED TO A LIST INSTEAD OF A DICT
# WHEN VALUES ARE CALCULATED DYNAMICALLY, NO NEED FOR DICT
# TREATED AS LIST UNTIL CLARIFIED
#

# <TODO> ID calculation system


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
        self._totalScoreUpdated = False
        self._totalScore = 0


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

    def getTotalScore(self):
        """
        Returns the sum of the value elements in the dictionary.
        Calculates dynamically.
        """
        if not self._totalScoreUpdated:
            self._totalScore = 0
            for value in collections.OrderedDict.itervalues(self):
                self._totalScore = self._totalScore + value
            self._totalScoreUpdated = True
        return self._totalScore

class Generation:

    def __init__(self, ID):
        self.ID = ID
        
        # Create internal variables for dynamic calculation
        self._totalScoreUpdated = False
        self._totalScore = 0

        self._sorted = False

        # Member dictionary
        self._members = []

    def addMember(self, member):
        self._totalScoreUpdated = False
        self._sorted = False
        self._members.append(member)

    def getMember(self, ID):
        return self._members[ID]

    # Not sure if this is a deep copy or not
    def getMembers(self):
        return list(self._members)

    def getTotalScore(self):
        """
        Returns the sum of the value elements in the dictionary.
        Calculates dynamically.
        """
        if not self._totalScoreUpdated:
            self._totalScore = 0
            for member in self._members:
                self._totalScore += member.getScore()
            self._totalScoreUpdated = True
        return self._totalScore

    # O(N^2), can be O(Nlog(N))
    # Higher complexity for simpler code
    def sortMembers(self):
        """
        Sorts the members of the dictionary.
        Calculates dynamically.
        """
        if not self._sorted:
            newList = [self._members[0]]
            members = list(self._members)
            for item in range(len(self._members)):
                maxIndex = self._maxMemberIndex(members)
                maxElement = members.pop(maxIndex)
                newList.append(maxElement)
            self._members = newList
            self._sorted = True

    # O(N)
    # Not calculated dynamically because members argument may not be self._members
    def _maxMemberIndex(self, members):
        max = members[0].getScore()
        maxIndex = 0
        index = 0
        for member in members[1:]:
            score = member.getScore()
            index += 1
            if score > max:
                max = score
                index = maxIndex
        if max == 0:
            raise Exception("Only negative scores in generation. Is this really an error?")
        return maxIndex
        

class Member(object):
    """
    Dictionary for storing entities for learning trials.
    Dynamically calculates all accessible properties.

    TODO:
    Create a copy() function.
    """

    def __init__(self, memberID=-1, generationID = -1, components={}):
        """
        Constructor for a LearningDictionary.
        Requires an ID to reference the dictionary.
        """
        self.memberID = memberID

        # The subsets of parameters which define the member
        # i.e. Edge, Node, Feedback vals for Controllers
        # How is this different from parameters for the start?
        self.components = components

        # List representing the Trials of this Member
        # Not populated until the member has been simulated
        self._trials = []

        # Score is calculated based on Trials of simulation results
        # Can be by average or maximum of Trial scores
        self._score = None

        if not "memberID" in self.components:
            print "Setting memberID to: " + str(memberID)
            self.components['memberID'] = memberID
        if not "generationID" in self.components:
            assert type(generationID) == type(1)
            print "Setting generationID to: " + str(generationID)
            self.components['generationID'] = generationID

    # Score encapsulated to indicate that it should not be altered by user
    def getScore(self):
        if not self._score:
            raise Exception("getScore called when score has not yet been assigned.")
        else:
            return self._score

    def copy(self):
        return Member(-1,self.parameters.copy())

    def _setScore(self, scoreMethod, trials):
        self._trials = trials

        # Scoring by average
        if scoreMethod == "average":
            sum = 0
            count = 0
            for trial in self._trials:
                sum += trial['score']
                count += 1
            self._score = sum / count

        # Scoring by maximum
        elif scoreMethod == "max":
            for trial in self._trials:
                self._score = max(self._score, trial['score'])

        # Raise exception on unknown scoring method
        else:
            raise Exception("MEMBER: <TODO>: Exception Hierarchy.")


class Controller(Member):
    
    DEFAULTS = {
        'nodeVals' : {
            'Min' : 0,
            'Max' : 1
            },
        'edgeVals' : {
            'Min' : 0,
            'Max' : 1
            },
        'feedbackVals' : {
            'Min'  : -1,
            'Max'  : 1
            },
        'goalVals' : {
            'Min' : -1,
            'Max' : 1
            }
        }

    def __init__(self, memberID=-1, generationID=-1, seedMember=None):
        assert type(generationID) == type(1)
        if seedMember:
            self._score = seedMember._score
            self._trials = seedMember._trials
            self.components = seedMember.components
        else:
            # This is necessary because calling the super constructor returns a pointer to the same instance
            # Why? Unknown. Investigating.
            self.memberID = memberID
            self.components = {}
            self._trials = []
            self._score = None

        if not "memberID" in self.components:
            print "Setting memberID to: " + str(memberID)
            self.components['memberID'] = memberID
        if not "generationID" in self.components:
            assert type(generationID) == type(1)
            print "Setting generationID to: " + str(generationID)
            self.components['generationID'] = generationID



class ParameterSet(Member):

    """
    Taken from TensegrityModel.cpp 1/28/2016

    default rod params
    double radius = 0.5;
    double density = 1.0;
    double friction = 1.0;
    double rollFriction = 0.0;
    double restitution = 0.2;

    default muscle params
    double stiffness = 1000.0;
    double damping = 10.0;
    double pretension = 0.0;
    double hist = false;
    double maxTens = 1000.0;
    double targetVelocity = 100.0;
    double minActualLength = 0.1;
    double minRestLength = 0.1;
    double rotation = 0;
    """

    DEFAULTS = {
        'tgRod' : {
            'Radius' : {
                'Min' : 0.1,
                'Max' : 0.9
                },
            'Density' : {
                'Min' : 0.1,
                'Max' : 1.0
                },
            'Friction' : {
                'Min' : 0.5,
                'Max' : 1.5
                },
            'RollFriction' : {
                'Min' : 0.0,
                'Max' : 1.0
                },
            'Restitution' : {
                'Min' : 0.1,
                'Max' : 0.3
                }
            },
        'tgBasicActuator' : {
            'Stiffness' : {
                'Min' : 500.0,
                'Max' : 1500.0
                },
            'Damping' : {
                'Min' : 5.0,
                'Max' : 15.0
                },
            'Pretension' : {
                'Min' : 0.0,
                'Max' : 1.0
                },
            # Hist boolean?
            'MaxTens' : {
                'Min' : 500.0,
                'Max' : 1500.0
                },
            'TargetVelocity' : {
                'Min' : 50.0,
                'Max' : 150.0
                },
            'MinActualLength' : {
                'Min' : 0.05,
                'Max' : 0.15
                },
            'MinRestingLength' : {
                'Min' : 0.05,
                'Max' : 0.15
                },
            'Rotation' : {
                'Min' : 0.0,
                'Max' : 0.1
                }
            }
        }

class Structure(Member):
    def __init__(self):
        return 0
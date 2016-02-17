import copy
import random

class Generation:

    _nextMemberID = 0
    _componentPopulations = {}
    _members = []

    def __init__(self, generationID):
        self._generationID = generationID

    def getID(self):
        return self._generationID

    def getComponentPopulations(self):
        return copy.deepcopy(self._componentPopulations)

    def addComponentPopulation(self, componentName, componentPopulation):
        for component in componentPopulation:
            self.addComponentMember(componentName, component)

    def addComponentMember(self, componentName, component):
        localComponent = copy.deepcopy(component)
        localComponent["generationID"] = self._generationID
        if not componentName in self._componentPopulations:
            self._componentPopulations[componentName] = []
        self._componentPopulations[componentName].append(localComponent)

    def _getNextMemberID(self):
        memberID = self._nextMemberID
        self._nextMemberID += 1
        return memberID

    def generateMemberFromComponents(self):
        memberID = self._getNextMemberID()
        newMember = Member(memberID=memberID, generationID=self._generationID)
        for componentName, componentPopulation in self._componentPopulations.iteritems():
            newMember.components[componentName] = copy.deepcopy(random.choice(componentPopulation))
        self._members.append(newMember)
        return newMember

    def getMembers(self):
        return copy.deepcopy(self._members)

class Member(object):

    # For later use
    _score = None

    def __init__(self, memberID, generationID):
        self.filePath = None
        self.memberID = memberID
        # TODO: Should components be made "pseudo-private"?
        self.components = {"memberID" : memberID, "generationID" : generationID}

    # Score mutator/accessor added for later use
    def getScore(self):
        if not self._score:
            raise Exception("getScore called when score has not yet been assigned.")
        else:
            return self._score

    def setScore(self, score):
        self._score = score

import os
from interfaces import NTRTMasterError

def dictionaryToList(dictionary):

    outputList = []

    for key in dictionary:
        value = dictionary[key]
        if type(value) == type({}):
            outputList.extend(dictionaryToList(value))
        elif type(value) == type(1):
            outputList.append(value)
        elif type(value) == type([]):
            outputList.extend(value)
        else:
            raise Exception("Trying to convert unrecognized dictionary element type into list.")

    return outputList

# Hack way of list->dictionary in O(N)
_list  = []
_index = 0
def listToDictionary(list, dictionary):
    _list = list
    outputDictionary = _listToDictionaryWorker(dictionary.copy())
    _list = []
    _index = 0
    return outputDictionary

def _listToDictionaryWorker(dictionary):
    outputDictionary = {}

    for key in dictionary:
        value = dictionary[key]
        if type(value) == type([]):
            for i in range(value):
                value[i] = _list[_index]
                _index += 1
        elif type(value) == type(1):
            outputDictionary[key] = _list[_index]
            _index += 1
        else:
            outputDictionary[key] = _listToDictionaryWorker(value)

    return outputDictionary

def printDict(dictionary, indent =0):
    assert type(dictionary) == type({})
    for key in dictionary:
        # print key
        value = dictionary[key]
        if type(value) == type({}):
            print "\t" * indent + key + ": "
            printDict(value, indent+1)
        elif type(value) != type([]):
            print "\t" * indent + key + ": " + str(value)
        elif type(value) == type([]):
            print "\t" * indent + key + ": ["
            for item in value:
                print "\t" * (indent+1) + str(item)
            print "\t" * indent + "]"
        else:
            raise Exception("Trying to convert unrecognized dictionary element type into list.")

def pause(text="..."):
    raw_input(text)

def tryMakeDir(dirPath):
    try:
        print "Making directory at: " + dirPath
        os.makedirs(dirPath)
    except OSError:
        if not os.path.isdir(dirPath):
            raise NTRTMasterError("Could not make directory at " + dirPath)

    def generationGenerator(self):
        return

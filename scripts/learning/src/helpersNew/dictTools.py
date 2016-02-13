import os
import json
import sys
import yaml
# from interfaces import NTRTMasterError

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
            raise Exception("Could not make directory at " + dirPath)

    def generationGenerator(self):
        return

def compareDictDeepType(dictA, dictB):
    sameDeepType = True
    if not dictA.keys() == dictB.keys():
        print "dictionaries have different keys:"
        print "dictA keys: " + str(dictA.keys())
        print "dictB keys: " + str(dictB.keys())
        sameDeepType = False
    else:
        # Fold type comparisons
        for key in dictA:
            sameDeepType &= compareDeepType(dictA[key], dictB[key])
    return sameDeepType

def compareListDeepType(listA, listB):
    sameDeepType = True
    if not len(listA) == len(listB):
        print "lists have different lengths:"
        print "listA len: " + str(len(listA))
        print "listB len: " + str(len(listB))
        sameDeepType = False
    else:
        index = 0

        # Importing from certain file formats may not preserve order
        # Sorting makes sure that elements are compared in the same order
        listASorted = list(listA)
        listASorted.sort()
        listBSorted = list(listB)
        listBSorted.sort()

        # Fold type comparisons
        while index < len(listASorted):
            sameDeepType &= compareDeepType(listASorted[index], listBSorted[index])
            index += 1

    return sameDeepType

def compareVarTypes(varA, varB):
    sameDeepType = True
    if not type(varA) == type(varB):
        sameDeepType = False
    return sameDeepType

def compareDeepType(varA, varB):
    sameDeepType = True
    dispatcher = {
        type([])  : compareListDeepType,
        type({})  : compareDictDeepType,
        type('a') : compareVarTypes,
        type(1)   : compareVarTypes,
        type(0.1) : compareVarTypes,
        type(u'a'): compareVarTypes
    }
    if type(varA) == type(varB):
        sameDeepType &= dispatcher[type(varA)](varA, varB)
    else:
        sameDeepType &= False

    return sameDeepType

def compareJSONDicts(jsonPathA, jsonPathB):
    sameDicts = True
    try:
        jsonFileA = open(jsonPathA, 'r')
        jsonDictA = json.load(jsonFileA)
        jsonFileA.close()

        jsonFileB = open(jsonPathB, 'r')
        jsonDictB = json.load(jsonFileB)
        jsonFileB.close()

        sameDicts &= compareDeepType(jsonDictA, jsonDictB)

    except IOError:
        raise Exception("Could not load one of the json files for comparing dictionaries.")

    return sameDicts

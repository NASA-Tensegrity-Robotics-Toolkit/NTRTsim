import os
import json
import yaml

def printDict(dictionary, indent=0):
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

def eqTypes(varA, varB):
    sameType = False
    if (type(varA) == type(0) or type(varA) == type(0.1)) and (type(varB) == type(0) or type(varB) == type(0.1)):
        sameType |= True
    else:
        sameType |= type(varA) == type(varB)
    return sameType

def loadFile(dictFilePath):
    file = open(dictFilePath, 'r')
    if ".json" in dictFilePath:
        dictionary = json.load(file)
    elif ".yaml" in dictFilePath:
        dictionary = yaml.load(file)
    else:
        raise Exception("Unknown file type in loadFile.")
    file.close()
    return dictionary

def dumpFile(contents, filePath):
    file = open(filePath, 'w')
    if ".json" in filePath:
        dictionary = json.dump(contents, file, indent=4)
    elif ".yaml" in filePath:
        dictionary = yaml.dump(contents, file)
    else:
        raise Exception("Unknown file type in dumpFile.")
    file.close()
    return dictionary

def deleteKeys(dictionary, badKeys):
    for key in badKeys:
        if key in dictionary.keys():
            dictionary.pop(key)

    for value in dictionary.values():
        if type(value) == type({}):
            deleteKeys(value, badKeys)

def compareDictDeepType(dictA, dictB):
    sameDeepType = True
    if not dictA.keys() == dictB.keys():
        print "dictionaries have different keys:"
        print "dictA keys: " + str(dictA.keys())
        print "dictB keys: " + str(dictB.keys())
        sameDeepType &= False
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
        sameDeepType &= False
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

# TODO: This function always returns true with how it is called. Refactor the dispatcher.
def compareVarTypes(varA, varB):
    sameDeepType = True
    if not eqTypes(varA, varB):
        print "type inconsistency:"
        print varA
        print varB
        sameDeepType &= False
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
    # Necessary to avoid indexing errors
    if eqTypes(varA, varB):
        sameDeepType &= dispatcher[type(varA)](varA, varB)
    else:
        print "type inconsistency:"
        print varA
        print varB
        sameDeepType &= False

    return sameDeepType

def compareFileDicts(pathA, pathB):
    sameDicts = True
    try:
        dictA = loadFile(pathA)
        dictB = loadFile(pathB)
        sameDicts &= compareDeepType(dictA, dictB)
    except IOError:
        raise Exception("Could not load one of the files for comparing dictionaries.")

    return sameDicts

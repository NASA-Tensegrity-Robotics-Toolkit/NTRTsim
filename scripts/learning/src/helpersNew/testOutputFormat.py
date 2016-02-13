import sys
import dictTools

if __name__ == "__main__":
    argv = sys.argv
    jsonFileA = argv[1]
    jsonFileB = argv[2]
    print "Testing json files for equivalent deep format:"
    print jsonFileA
    print jsonFileB
    if dictTools.compareJSONDicts(jsonFileA, jsonFileB):
        print "Files are of equiavlent deep format."
    else:
        print "~~~ Files are NOT of equivalent deep format. ~~~"
    exit(0)
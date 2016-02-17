import sys
import dictTools

if __name__ == "__main__":
    argv = sys.argv
    fileA = argv[1]
    fileB = argv[2]
    print "Testing json files for equivalent deep format:"
    print fileA
    print fileB
    if dictTools.compareFileDicts(fileA, fileB):
        print "Files are of equiavlent deep format."
    else:
        print "~~~ Files are NOT of equivalent deep format. ~~~"
""" Module for Porting Tensegrity Data in JSON format (Proof of Concept) """
" As an example, this module functions by incrementing the arbitrary field "
" 'x' in the provided JSON file "

import sys, json
from pprint import pprint

""" change the state of tensegrity tg """
def tweak(tg):
    tg['x'] += 1
    pprint(tg)
    return tg

def import_tg_data(filename):
    tgJSON = open(filename, 'r')
    tg = json.load(tgJSON)
    tg_tweaked = tweak(tg)
    tgJSON.close()
    return tg_tweaked

def export_tg_data(filename, json_data):
    tgJSON = open('tgData.json', 'w')
    json.dump(json_data, tgJSON, indent=4)
    tgJSON.close()
    return

def main(f):
    tweaked_json_data = import_tg_data(f)
    export_tg_data(f, tweaked_json_data)

if __name__ == '__main__':
    if (len(sys.argv) > 1):
        main(sys.argv[1])
    else:
        print "Error: no JSON file passed to tgPort.py"

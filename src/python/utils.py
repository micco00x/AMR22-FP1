import json

def json2dict(json_file):
    dict = {}
    with open(json_file) as file:
        dict = json.load(file)
    return dict
        
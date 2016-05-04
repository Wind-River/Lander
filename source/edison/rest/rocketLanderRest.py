#!/usr/bin/python

import requests
import json
import sys

# will only take one argument
url = 'http://128.224.186.83:3000/api/lander'

if len(sys.argv) > 1 :
    data = json.loads(sys.argv[1])
    print "Data to Enter:"
    print json.dumps(data, indent=4, sort_keys=True)
    response = requests.post(url,json=data)
    print response.text
else :
    print "Get\n"
    response = requests.get(url)
    parsedJson = json.loads(response.text)
    print json.dumps(parsedJson, indent=4, sort_keys=True)



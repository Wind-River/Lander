#!/usr/bin/python

import requests
import json
import sys

# will only take one argument
url = 'http://128.224.186.83:3000/api/lander'

DEBUG = False

if len(sys.argv) > 1 :
    data = json.loads(sys.argv[1])
    response = requests.post(url,json=data)

    if DEBUG:
    	print "Data to Enter:"
    	print json.dumps(data, indent=4, sort_keys=True)
    	print "Response: " + response.text
else :
    response = requests.get(url)
    if DEBUG:
    	print "Get\n"
	    parsedJson = json.loads(response.text)
	    print json.dumps(parsedJson, indent=4, sort_keys=True)
	else
    	print response.txt



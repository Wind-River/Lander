#!/usr/bin/python

#
#  Copyright (c) 2016 Wind River Systems, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#

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
    else:
    	print response.text



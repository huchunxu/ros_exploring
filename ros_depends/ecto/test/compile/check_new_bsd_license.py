#!/usr/bin/env python

import sys

bsd = '''
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
'''

skip_check_tag = "Willow Garage BSD License not applicable"

nerrors = 0

import os

autofix = False
if "ECTO_LICENSE_AUTOFIX" in os.environ:
    autofix = True

files = sys.argv[1:]
    
commentchars = { '.cpp' : '//',
                 '.hpp' : '//',
                 '.py'  : '#',
                 '.cmake' : '#',
                 '.txt' : '#'
                 }
                 

for filename in files:

    txt = open(filename).read()

    thiserror = False
    result = filename + "..."
    if skip_check_tag in txt:
        result += "ok"
    else:
        for l in bsd.split('\n'):
            if l not in txt:
                result += "missing: " + l + "\n"
                thiserror = True
        if thiserror:
            nerrors += 1
        else:
            result += "ok"

    if thiserror and autofix:
        newf = open(filename, "w")
        for k, v in commentchars.iteritems():
            if filename.endswith(k):
                cmt = v
        if txt.startswith('#!'):
            hashbang, rest = txt.split('\n', 1)
            print >>newf, hashbang
        else:
            rest = txt
        print >>newf, cmt, bsd.replace('\n', '\n' + cmt + ' ')
        print >>newf, rest
        newf.close()
        result += filename + "AUTOFIXED"
    print result


sys.exit(nerrors)

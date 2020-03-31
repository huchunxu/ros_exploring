import subprocess,sys

tags = ('0.1.2',
        '0.1.3',
        '0.2.0',
        '0.3.0',
        '0.3.1',
        '0.3.2',
        '0.3.3',
        '0.3.4',
        "HEAD"
        )

pairs = []
for i in range(0,len(tags)-1):
    pairs += [(tags[i],tags[i+1])]
pairs.reverse()
print '''
Changelog
=========
This is a generated changelog file.
'''
link = '`%(ref)s <https://github.com/plasmodic/ecto/commit/%(ref)s>`_'

fclen = len(link % dict(ref='*' * 7))

headline = '=' * fclen + " " + "="*1024
fmt = link + " %(comment)s"

#headline = fmt % dict(ref='='*20, comment = 

for tag1,tag2 in pairs:
    header = '%s..%s'%(tag1,tag2)
    p = subprocess.Popen(['git',
                       'log',
                       '--oneline',
                       '--no-color',
                       '%s..%s'%(tag1,tag2)], stdout=subprocess.PIPE)
    lines = [x.split(' ',1) for x in p.communicate()[0].strip().split('\n') ]
    # '* `%(ref)s <https://github.com/plasmodic/ecto/commit/%(ref)s>`_ %(comment)s` %s'%
    lines = [fmt % dict(ref=x[0],comment=x[1]) for x in lines
             if len(x) > 1]
    if len(lines) > 0:
        print header + '\n' + '^' * len(header) + '\n'
        print headline
        print '\n'.join(lines)
        print headline
    print '\n'

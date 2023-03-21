import os
import re
import string
import subprocess
import imgtool

def _get_version_from_tag():
    '''
    Get the version from a git tag. Only if this exact git revision is tagged will the version be
    returned.
    '''
    result = subprocess.run(['git', 'describe', '--exact-match'], stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, universal_newlines=True)
    
    if (result.stdout):
        return (result.stdout)
    else:
        return ('1.1.2')

'''
Creates the versioning builder which can fill in the version info into a template.

It also saves some version info already in the environment so the builder can track changes on
that info and build again when needed.
'''

if os.path.exists('../../../.git'):
    git_revision = subprocess.run(['git', 'rev-parse', 'HEAD'], stdout=subprocess.PIPE,
            universal_newlines=True).stdout.rstrip()
    i = int(git_revision[0:8], 16)
    git_revision = str(i)
    version = _get_version_from_tag()
else:
    git_revision = ''
    version = '1.1.2'
    
bash_cmd ="python ./imgtool.py sign -v 1.1.2 --header-size 0x400 --align 8 --max-align 8 --slot-size 0x28000 --max-sectors 2 --pad --pad-header ../Objects/gnr2_ra6t2.hex ../output/gnr2_ra6t2_Signed.hex"
print(bash_cmd)
process =subprocess.run(bash_cmd.split(), stdout=subprocess.PIPE,
           stderr=subprocess.DEVNULL, universal_newlines=True)
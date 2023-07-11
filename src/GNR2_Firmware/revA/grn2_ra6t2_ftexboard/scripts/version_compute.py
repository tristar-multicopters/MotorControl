import os
import re
import string
import subprocess
import imgtool

def _get_gnr_version_from_tag():
    '''
    Get the version from a git tag. It will retrieve the nearest Renesas Tag 
    '''
    result = subprocess.run(['git', 'describe','--tags','--match', 'R_*','--abbrev=0'], stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, universal_newlines=True)
    
    if (result.stdout):
        result.stdout = result.stdout.rstrip()
        return (result.stdout.replace('R_',''))
    else:
        return ('1.0.0')

def _get_dfu_pack_version_from_tag():
    '''
    Get the version from a git tag. Will retrieve the nearest PACK Tag, should be this commit
    '''
    result = subprocess.run(['git', 'describe','--tags','--match', 'PACK_*','--abbrev=0'], stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, universal_newlines=True)
    
    if (result.stdout):
        result.stdout = result.stdout.rstrip()
        return (result.stdout.replace('PACK_',''))
    else:
        return ('0.0.0')
        
def convert_tag_to_version_pack(version_string):


    # Split the version string into major, minor, and revision parts
    major, minor, revision = map(int, version_string.split('.'))

    # Convert the version parts to hexadecimal in little endian
    hex_number = (revision << 16) | (minor << 8) | major

    # Print the result in hex format
    return(str(hex_number))

'''
Creates the versioning builder which can fill in the version info into a template.

It also saves some version info already in the environment so the builder can track changes on
that info and build again when needed.
'''

'''
Retrieves version for generating the mcuboot img
'''

if os.path.exists('../../../.git'):
    version = _get_renesas_version_from_tag()
else:
    version = '1.0.0'

version_pack = _get_dfu_pack_version_from_tag()
dfu_pack = convert_tag_to_version_pack(version_pack)
    
bash_cmd ="python ./imgtool.py sign -v "+version+ "+" + dfu_pack +" --header-size 0x400 --align 8 --max-align 8 --slot-size 0x28000 --max-sectors 2 --pad --pad-header ../Objects/gnr2_ra6t2.hex ../output/gnr2_ra6t2_Signed.hex"
print(bash_cmd)

print ('GNR  ',version)
print ('PACK     ',version_pack)

process =subprocess.run(bash_cmd.split(), stdout=subprocess.PIPE,
           stderr=subprocess.DEVNULL, universal_newlines=True)
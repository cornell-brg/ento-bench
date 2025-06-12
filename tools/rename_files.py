import glob
import subprocess
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument( 'path', help = 'path to folder containing files to rename' )
args = parser.parse_args()

os.chdir( args.path )
new_name = args.path.strip( '/' ).split( '/' ) [-1]

files = glob.glob( '*' )
for file in files:
    ext = file.split( '.' )[-1]
    subprocess.run( [ 'mv', file, new_name + '.' + ext ] )


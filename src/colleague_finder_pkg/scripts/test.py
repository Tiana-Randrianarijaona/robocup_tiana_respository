import sys
import os

# Get the path to the ../include directory
include_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../include'))

# Add that directory to the sys.path
sys.path.append(include_path)

# Now you can import the class
from ColleagueFinder import ColleagueFinder

import sys, os

# allow importing backend
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from backend.api import app

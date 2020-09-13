from src.common import Atmosphere
from src.common.Rotations import ned_to_body
from src.common import Earth
from numpy import pi

b = Earth(90000)
print(b.gravity())


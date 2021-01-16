from common import Atmosphere
from test.test_library import is_close

h_0 = Atmosphere(0)
h_40k = Atmosphere(40000)

out = list()
out.append((is_close(h_0.air_density(), 0.002379)))
out.append(is_close(h_40k.air_density(), 0.0005848))
out.append((is_close(h_0.pressure(), 2118.145)))
out.append(is_close(h_40k.pressure(), 391.234))
out.append((is_close(h_0.speed_of_sound(), 1116.462)))
out.append(is_close(h_40k.speed_of_sound(), 967.723))
out.append((is_close(h_0.viscosity(), 3.737e-07)))
out.append(is_close(h_40k.viscosity(), 2.967e-07))

if any(out):
    print("atmosphere test passed!")
else:
    print("atmosphere test failed")

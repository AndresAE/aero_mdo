from src.common import Earth
from test.test_library import is_close

h_0 = Earth(0)
h_40k = Earth(40000)

out = list()
out.append((is_close(h_0.gravity(), 32.14894)))
out.append(is_close(h_40k.gravity(), 32.026389))

if any(out):
    print("earth test passed!")
else:
    print("earth test failed")

from src.common.rotations import angular_rate_rotation, body_to_wind, eci_to_ned, ned_to_body
from numpy import pi
from test.test_library import is_close

e2b = angular_rate_rotation(5 * pi/180, 5 * pi/180)
b2w = body_to_wind(5 * pi/180, 5 * pi/180)
e2n = eci_to_ned(5 * pi/180, 5 * pi/180)
n2b = ned_to_body(5 * pi/180, 5 * pi/180, 5 * pi/180)

out = list()
out.append(is_close(e2b[0, 0], 1))
out.append(is_close(e2b[1, 1], 0.99619))
out.append(is_close(e2b[2, 2], 1))
out.append(is_close(b2w[0, 0], 0.9924))
out.append(is_close(b2w[1, 1], 0.99619))
out.append(is_close(b2w[2, 2], 0.99619))
out.append(is_close(e2n[0, 0], 0.99619))
out.append(is_close(e2n[1, 1], 0.99619))
out.append(is_close(e2n[2, 2], 0.9924))
out.append(is_close(n2b[0, 0], 0.9924))
out.append(is_close(n2b[1, 1], 0.99306))
out.append(is_close(n2b[2, 2], 0.9924))

if any(out):
    print("rotation test passed!")
else:
    print("rotation test failed")

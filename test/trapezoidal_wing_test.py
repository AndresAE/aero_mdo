from src.modeling.trapezoidal_wing import mac, root_chord, span, sweep_x, x_mac, y_chord, y_mac
from test.test_library import is_close
ar = 5
s = 10
sweep_le = 30
taper = 0.5

c_bar = mac(ar, s, taper)
cr = root_chord(ar, s, taper)
b = span(ar, s)
sweep = sweep_x(ar, taper, sweep_le, 0.5)
y = y_mac(ar, s, taper)
x = x_mac(3, sweep_le)
c = y_chord(0.25, cr, b, taper)

out = list()
out.append((is_close(c_bar, 1.46659)))
out.append(is_close(b, 7.071067))
out.append(is_close(cr, 1.8856))
out.append(is_close(c, 1.81895))
out.append(is_close(x, 1.73205))
out.append(is_close(y, 1.5713))
out.append(is_close(sweep, 23.942))

if any(out):
    print("trapezoidal wing test passed!")
else:
    print("trapezoidal wing test failed")

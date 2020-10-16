"""return plotted airliner data from data>airliner_data.csv."""
import csv
from matplotlib import pyplot as plt
from numpy import array
import os

cur_path = os.path.dirname(__file__)
os.chdir("..")
os.chdir("..")

filename = 'data\\airliner_data.csv'
# read airliner data csv
results = []
with open(filename) as csv_file:
    reader = csv.reader(csv_file)  # change contents to floats
    for row in reader:  # each row is a list
        results.append(row)

airplanes = array(results)

# group aircraft types
# wide body
plane = airplanes[airplanes[:, 9] == 'turbofan', :]
wide = plane[plane[:, 8] == 'wide', :]

# narrow body
plane = airplanes[airplanes[:, 9] == 'turbofan', :]
narrow = plane[plane[:, 8] == 'narrow', :]

# regional jet
plane = airplanes[airplanes[:, 9] == 'turbofan', :]
regional_jet = plane[plane[:, 8] == 'regional', :]

# regional turboprop
plane = airplanes[airplanes[:, 9] == 'turboprop', :]
regional_prop = plane[plane[:, 8] == 'regional', :]

wing_area = {
    'wide': [float(x) for x in wide[:, 6]],
    'narrow': [float(x) for x in narrow[:, 6]],
    'regional_jet': [float(x) for x in regional_jet[:, 6]],
    'regional_prop': [float(x) for x in regional_prop[:, 6]],
}
mtow = {
    'wide': [float(x) for x in wide[:, 2]],
    'narrow': [float(x) for x in narrow[:, 2]],
    'regional_jet': [float(x) for x in regional_jet[:, 2]],
    'regional_prop': [float(x) for x in regional_prop[:, 2]],
}
ew = {
    'wide': [float(x) for x in wide[:, 3]],
    'narrow': [float(x) for x in narrow[:, 3]],
    'regional_jet': [float(x) for x in regional_jet[:, 3]],
    'regional_prop': [float(x) for x in regional_prop[:, 3]],
}
pax = {
    'wide': [float(x) for x in wide[:, 5]],
    'narrow': [float(x) for x in narrow[:, 5]],
    'regional_jet': [float(x) for x in regional_jet[:, 5]],
    'regional_prop': [float(x) for x in regional_prop[:, 5]],
}
r = {
    'wide': [float(x) for x in wide[:, 7]],
    'narrow': [float(x) for x in narrow[:, 7]],
    'regional_jet': [float(x) for x in regional_jet[:, 7]],
    'regional_prop': [float(x) for x in regional_prop[:, 7]],
}
speed = {
    'wide': [float(x) for x in wide[:, 4]],
    'narrow': [float(x) for x in narrow[:, 4]],
    'regional_jet': [float(x) for x in regional_jet[:, 4]],
    'regional_prop': [float(x) for x in regional_prop[:, 4]],
}
deliveries = {
    'wide': [float(x) for x in wide[:, 13]],
    'narrow': [float(x) for x in narrow[:, 13]],
    'regional_jet': [float(x) for x in regional_jet[:, 13]],
    'regional_prop': [float(x) for x in regional_prop[:, 13]],
}
price = {
    'wide': [float(x) for x in wide[:, 10]],
    'narrow': [float(x) for x in narrow[:, 10]],
    'regional_jet': [float(x) for x in regional_jet[:, 10]],
    'regional_prop': [float(x) for x in regional_prop[:, 10]],
}

# plotting
# x_fit = array([0, 500, 5000])
plt.figure(figsize=(12, 7))
plt.plot(wing_area['wide'], mtow['wide'], 'ro', label='wide')
plt.plot(wing_area['narrow'], mtow['narrow'], 'bo', label='narrow')
plt.plot(wing_area['regional_jet'], mtow['regional_jet'], 'go', label='regional jet')
plt.plot(wing_area['regional_prop'], mtow['regional_prop'], 'ko', label='regional prop')
# coef = polyfit(wing_area['regional_prop'], mtow['regional_prop'], 1)
# y_fit = polyval(coef, x_fit)
# plt.plot(x_fit, y_fit, 'y--')
plt.legend()
plt.xlabel('Wing Area [ft2]')
plt.ylabel('Max Takeoff Weight [lbs]')
plt.grid(True)
plt.show()

# x_fit = linspace(0, 5000, 50)
plt.figure(figsize=(12, 7))
plt.plot(wing_area['wide'], ew['wide'], 'ro', label='wide')
plt.plot(wing_area['narrow'], ew['narrow'], 'bo', label='narrow')
plt.plot(wing_area['regional_jet'], ew['regional_jet'], 'go', label='regional jet')
plt.plot(wing_area['regional_prop'], ew['regional_prop'], 'ko', label='regional prop')
# coef = polyfit(wing_area['regional_prop'], ew['regional_prop'], 1)
# y_fit = polyval(coef, x_fit)
# plt.plot(x_fit, y_fit, 'y--')
plt.legend()
plt.grid(True)
plt.xlabel('Wing Area [ft2]')
plt.ylabel('Empty Weight [lbs]')
plt.show()

# x_fit = array([0, 5000, 50000, 100000, 200000, 300000, 400000, 500000])
plt.figure(figsize=(12, 7))
plt.plot(mtow['wide'], ew['wide'], 'ro', label='wide')
plt.plot(mtow['narrow'], ew['narrow'], 'bo', label='narrow')
plt.plot(mtow['regional_jet'], ew['regional_jet'], 'go', label='regional jet')
plt.plot(mtow['regional_prop'], ew['regional_prop'], 'ko', label='regional prop')
# coef = polyfit(mtow['regional_prop'], ew['regional_prop'], 1)
# y_fit = polyval(coef, x_fit)
# plt.plot(x_fit, y_fit, 'y--')
plt.legend()
plt.grid(True)
plt.xlabel('Max Takeoff Weight [lbs]')
plt.ylabel('Empty Weight [lbs]')
plt.show()

# x_fit = array([0, 10, 20, 100, 200, 300, 500, 600])
plt.figure(figsize=(12, 7))
plt.plot(pax['wide'], ew['wide'], 'ro', label='wide')
plt.plot(pax['narrow'], ew['narrow'], 'bo', label='narrow')
plt.plot(pax['regional_jet'], ew['regional_jet'], 'go', label='regional jet')
plt.plot(pax['regional_prop'], ew['regional_prop'], 'ko', label='regional prop')
# coef = polyfit(pax['regional_prop'], ew['regional_prop'], 1)
# y_fit = polyval(coef, x_fit)
# plt.plot(x_fit, y_fit, 'y--')
plt.legend()
plt.grid(True)
plt.xlabel('PAX')
plt.ylabel('Empty Weight [lbs]')
plt.show()

# x_fit = array([0, 500, 5000, 7000, 10000])
plt.figure(figsize=(12, 7))
plt.plot(r['wide'], ew['wide'], 'ro', label='wide')
plt.plot(r['narrow'], ew['narrow'], 'bo', label='narrow')
plt.plot(r['regional_jet'], ew['regional_jet'], 'go', label='regional jet')
plt.plot(r['regional_prop'], ew['regional_prop'], 'ko', label='regional prop')
# coef = polyfit(r['regional_prop'], ew['regional_prop'], 1)
# y_fit = polyval(coef, x_fit)
# plt.plot(x_fit, y_fit, 'y--')
plt.legend()
plt.grid(True)
plt.xlabel('Range [NM]')
plt.ylabel('Empty Weight [lbs]')
plt.show()

# x_fit = array([0, 50, 100, 150, 200, 300, 400, 500, 600])
plt.figure(figsize=(12, 7))
plt.plot(speed['wide'], r['wide'], 'ro', label='wide')
plt.plot(speed['narrow'], r['narrow'], 'bo', label='narrow')
plt.plot(speed['regional_jet'], r['regional_jet'], 'go', label='regional jet')
plt.plot(speed['regional_prop'], r['regional_prop'], 'ko', label='regional prop')
# coef = polyfit(speed['regional_prop'], r['regional_prop'], 1)
# y_fit = polyval(coef, x_fit)
# plt.plot(x_fit, y_fit, 'y--')
plt.legend()
plt.grid(True)
plt.xlabel('Cruise Speed [KTAS]')
plt.ylabel('Range [NM]')
plt.show()

# x_fit = array([0, 500, 5000, 7000, 10000])
plt.figure(figsize=(12, 7))
plt.plot(r['wide'], array(mtow['wide']) / array(ew['wide']), 'ro', label='wide')
plt.plot(r['narrow'], array(mtow['narrow']) / array(ew['narrow']), 'bo', label='narrow')
plt.plot(r['regional_jet'], array(mtow['regional_jet']) / array(ew['regional_jet']), 'go', label='regional jet')
plt.plot(r['regional_prop'], array(mtow['regional_prop']) / array(ew['regional_prop']), 'ko', label='regional prop')
# coef = polyfit(r['regional_prop'], array(mtow['regional_prop']) / array(ew['regional_prop']), 1)
# y_fit = polyval(coef, x_fit)
# plt.plot(x_fit, y_fit, 'y--')
plt.legend()
plt.grid(True)
plt.xlabel('Range [NM]')
plt.ylabel('Max Takeoff Weight/Empty Weight')
plt.ylim((0, 3))
plt.show()

# x_fit = array([0, 10, 20, 100, 200, 300, 500, 600])
plt.figure(figsize=(12, 7))
plt.plot(pax['wide'], r['wide'], 'ro', label='wide')
plt.plot(pax['narrow'], r['narrow'], 'bo', label='narrow')
plt.plot(pax['regional_jet'], r['regional_jet'], 'go', label='regional jet')
plt.plot(pax['regional_prop'], r['regional_prop'], 'ko', label='regional prop')
# coef = polyfit(pax['regional_prop'], r['regional_prop'], 1)
# y_fit = polyval(coef, x_fit)
# plt.plot(x_fit, y_fit, 'y--')
plt.legend()
plt.grid(True)
plt.xlabel('PAX')
plt.ylabel('Range [NM]')
plt.show()

plt.figure(figsize=(12, 7))
plt.plot(ew['wide'], price['wide'], 'ro', label='wide')
plt.plot(ew['narrow'], price['narrow'], 'bo', label='narrow')
plt.plot(ew['regional_jet'], price['regional_jet'], 'go', label='regional jet')
plt.plot(ew['regional_prop'], price['regional_prop'], 'ko', label='regional prop')
plt.legend()
plt.grid(True)
plt.xlabel('Empty Weight [lbs]')
plt.ylabel('Price [$]')
plt.show()

plt.figure(figsize=(12, 7))
plt.plot(mtow['wide'], price['wide'], 'ro', label='wide')
plt.plot(mtow['narrow'], price['narrow'], 'bo', label='narrow')
plt.plot(mtow['regional_jet'], price['regional_jet'], 'go', label='regional jet')
plt.plot(mtow['regional_prop'], price['regional_prop'], 'ko', label='regional prop')
plt.legend()
plt.grid(True)
plt.xlabel('Max Takeoff Weight [lbs]')
plt.ylabel('Price [$]')
plt.show()


plt.figure(figsize=(12, 7))
k = 100
ii = 0
for iw in mtow['wide']:
    plt.plot(speed['wide'][ii], r['wide'][ii],
             'ro', markersize=deliveries['wide'][ii]/k)
    ii = 1 + ii
ii = 0
for iw in mtow['narrow']:
    plt.plot(speed['narrow'][ii], r['narrow'][ii],
             'bo', markersize=deliveries['narrow'][ii]/k)
    ii = 1 + ii
ii = 0
for iw in mtow['regional_jet']:
    plt.plot(speed['regional_jet'][ii], r['regional_jet'][ii],
             'go', markersize=deliveries['regional_jet'][ii]/k)
    ii = 1 + ii
ii = 0
for iw in mtow['regional_prop']:
    plt.plot(speed['regional_prop'][ii], r['regional_prop'][ii],
             'ko', markersize=deliveries['regional_prop'][ii]/k)
    ii = 1 + ii
plt.grid(True)
plt.xlabel('Cruise Speed [KTAS]')
plt.ylabel('Range [NM]')
plt.show()

plt.figure(figsize=(12, 7))
k = 100
ii = 0
for iw in mtow['wide']:
    plt.plot(pax['wide'][ii], r['wide'][ii],
             'ro', markersize=deliveries['wide'][ii]/k)
    ii = 1 + ii
ii = 0
for iw in mtow['narrow']:
    plt.plot(pax['narrow'][ii], r['narrow'][ii],
             'bo', markersize=deliveries['narrow'][ii]/k)
    ii = 1 + ii
ii = 0
for iw in mtow['regional_jet']:
    plt.plot(pax['regional_jet'][ii], r['regional_jet'][ii],
             'go', markersize=deliveries['regional_jet'][ii]/k)
    ii = 1 + ii
ii = 0
for iw in mtow['regional_prop']:
    plt.plot(pax['regional_prop'][ii], r['regional_prop'][ii],
             'ko', markersize=deliveries['regional_prop'][ii]/k)
    ii = 1 + ii
plt.grid(True)
plt.xlabel('PAX')
plt.ylabel('Range [NM]')
plt.show()

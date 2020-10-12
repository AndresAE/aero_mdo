# aero_mdo
Aircraft Multidisciplinary Design Software

#Supports:
Subsonic, fixed-wing aircraft. Jet, and propeller propulsion systems. 
Rigid aircraft assumption, but this will be addressed in future releases.


# Package Requirements
Download python 3.7 (https://www.python.org/downloads/release/python-370/) or higher and install to path. The following python packages are required: control, 
numpy, matplotlib, pip, scipy, avlwrapper. I recommend using pycharm IDE, as it simplifies package install, and code debugging. Will need a LaTex compiler to edit documentation.
You will need to add avl.exe (http://web.mit.edu/drela/Public/web/avl/) to path. A copy of avl.exe is included in this repository.

# To-Do
## Short-Term
independent propulsion system masses
## Long-Term
Add unit tests for aircraft, fuselage class.
Add unit test for longitudinal and lateral directional analysis method libraries.
Automate unit testing.
Add build-up inertia capabilities.
Finish Design scripting capabilities.
Add parametric FEA and FTR ratio to aero model.
Add auto-grid generation.
Add inviscid CFD (Cart3d) to aero modeling capabilities.
Add viscous CFD (Fun3d) to aero modeling capabilities.

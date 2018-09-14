There are three projects:
- endeffector (simulation of an endeffector system)
- nnpp (main part, includes implementation of the neuron model, optimization algorithm, simulator for the rover and NPark) make depends on endeffector library
- narm (narm) make depends on nnpp library
- arm_full (parts of the Model reduction + code to create the sensory and motor encodings for NArm) make depends on nnpp
- snr (noise attack experiments) make depends on nnpp

To build the most important projects run:

cd endeffector
make lib
cd ..
cd nnpp
make
make lib
cd ..
cd narm
make
cd ..


Note:
The code in this repository is intended for the cyclic design and optimization process of engineering of a wormnet controller.

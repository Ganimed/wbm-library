# iCub Nancy models

To better deal with local modifications of iCub Nancy model, the file model.urdf has been duplicated. In particular:

- model.urdf --> it is the file loaded in yarpWholeBodyInterface.ini and is the urdf that can be found in https://github.com/robotology-playground/icub-models/tree/master/iCub/robots/iCubNancy01, plus two additional frames for the upper legs (see also https://github.com/robotology-playground/icub-model-generator/issues/42);

- model_STANDUP.urdf --> it is the model used for the standup demo. For the time being, it is identical two model.urdf;

- model_BALANCING_IIT.urdf --> it is the model used for YOGA++ demo (identical to the others, but without the upper legs frames)


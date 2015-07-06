Tensegrity Elbow Project
-----------------

ScarrArmModel
    - Originally based upon Dr. Graham Scarr's designs for a passive elbow 
    tensegrity structure
    - Three compression elements (humerus, ulna, olecranon analogs)
    - Eighteen tension elements (brachioradialis, anconeus, pronator teres, fascial tissue analogs)
    - Uses tgBasicActuator class for cables (tension elements)

ScarrArmController
    - Actuates groups of tension elements at a time according to their tags (i.e. brachioradialis, anconeus etc.)
    - Plug n' play control achieved by modifying onStep() function


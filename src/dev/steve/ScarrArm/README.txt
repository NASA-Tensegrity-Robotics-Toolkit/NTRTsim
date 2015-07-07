Tensegrity Elbow Project
-----------------

ScarrArmModel
    - Originally based upon Dr. Graham Scarr's designs for a passive elbow 
      tensegrity structure
    - Three compression elements which are analogous in function to: humerus, 
      ulna, olecranon
    - Eighteen tension elements which are analogous in function to: 
      brachioradialis, anconeus, opposing teres, other tendons and ligaments
    - Uses tgBasicActuator class for cables (tension elements)

ScarrArmController
    - Actuates groups of tension elements at a time according to their tags 
      (i.e. brachioradialis, anconeus etc.)
    - Plug n' play control achieved by modifying onStep() function


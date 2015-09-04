Tensegrity Upper Limb Project
-----------------

UpperLimbTriangleModel
    - Models the radius/ulna, huerus, clavicle, and scapula of an arm
      as a tensegrity
    - Uses individual contact cables as muscles
    - Incorporates tetrahedron design (Erik Jung) 

UpperLimbController
    - Actuates groups of tension elements at a time according to their tags 
      (i.e. brachioradialis, anconeus etc.)
    - Plug n' play control achieved by modifying onStep() function


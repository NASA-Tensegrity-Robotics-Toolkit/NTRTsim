.. highlight:: yaml

YAML Model Builder
=========================================

The YAML model builder was built to allow users to easily create, modify and run tensegrity structures using NTRT’s simulator. Contrary to building structures in C++, using the YAML model builder does not require any programming knowledge, it does not require any knowledge of NTRT’s libraries and does not require a compilation step each time a modification to a structure is made. Structures defined in YAML are also considerably shorter and simpler to create than their C++ equivalents. YAML is a human-readable data serialization format. It was chosen because of its simplicity and clean syntax.

Building Simple Structures
-----------------------------------------

Shown below is a 3-Prism structure defined in YAML. It is important to note that spacing and indentation are important in YAML and inconsistent spacing will cause errors when parsing the file. Using a text editor such as Notepad++ or Sublime Text is recommended when defining structures. The site http://codebeautify.org/yaml-validator can be used to make sure the structure you have defined is valid YAML before running it in the simulator.
::

  nodes:
    bottom1: [-5, 0, 0]
    bottom2: [5, 0, 0]
    bottom3: [0, 0, 8.66]

    top1: [-5, 5, 0]
    top2: [5, 5, 0]
    top3: [0, 5, 8.66]

  pair_groups:
    rod:
      - [bottom1, top2]
      - [bottom2, top3]
      - [bottom3, top1]

    string:
      - [bottom1, bottom2]
      - [bottom2, bottom3]
      - [bottom1, bottom3]

      - [top1, top2]
      - [top2, top3]
      - [top1, top3]

      - [bottom1, top1]
      - [bottom2, top2]
      - [bottom3, top3]

Running Structures in the Simulator
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before running a structure in the simulator, make sure you have completed all the setup/installation steps for NTRT. The YAML model builder is run by running the BuildModel executable (located in the yamlbuilder directory) and providing it with the path to a YAML structure as a command line argument. To run the structure above in the simulator go to the root NTRT directory and type in:
::

  build/yamlbuilder/BuildModel resources/YAMLStructures/BaseStructures/3Prism.yaml

Adding Nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Tensegrity structures are made up of rods and strings. The vertices of the rods and strings are defined using the keyword “nodes”. Each node is defined by a key-value pair where the key is a unique name and the value is a 3-element array with coordinates of the node. Names should not contain spaces, periods, or slash characters. The x and z coordinate axes are horizontal and run parallel to the ground (with the z axis representing depth). The y coordinate axis is vertical and perpendicular to the ground.
::

  nodes:
    bottom1: [-5, 0, 0]


Adding Pairs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Rods and strings are defined using the “pair_groups” keyword and are grouped by tags. Tags determine how each pair is built in the simulator. There are different types of rods and strings that can be used to build any given pair. These are defined using builders which will be discussed in more detail in the next section. Default builders, that define a default rod and string, are added automatically to match the tags “rod” and “string” so that a tensegrity structure can be defined without having to worry about builders. Each tag holds a list of one or more pairs, where each pair is defined by the two nodes it connects. If a tag has a space such as “prism string” it is treated as two different tags that are assigned to a given pair. Tags should not use any period or slash characters.
::

  pair_groups:
    rod:
      - [bottom1, top2]


Adding Builders
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Shown below is an example of builders that can be used to modify how rods and strings are built in the simulator.
::

  builders:
    rod:
      class: tgRodInfo
      parameters:
        density: 0.688
        radius: 0.31
    string:
      class: tgBasicActuatorInfo
      parameters:
        stiffness: 1000
        damping: 10
        pretension: 1000

The “builders” keyword is used to define one or more builders. Builders work through tag matching. For example, the builder “rod” will match any pairs with the tag “rod”. The builder “rod” will also match any pairs with the tag “prism rod” since “prism rod” is treated as two different tags and the “rod” builder is looking for any pairs that include the tag “rod”. As was mentioned earlier, the YAML model builder automatically adds a default “rod” and “string” builder to make building structures even faster. If a “rod ”or “string” builder is defined by the user, it will override the default “rod” and “string” builder. Builders defined inside one file should never overlap (eg. using a “muscle” and “leg muscle” builder in the same file).

Each builder tag needs to be given a class using the “class” keyword. The class determines the properties of the rod or string. The basic rod and string classes are tgRodInfo and tgBasicActuatorInfo. More information about different string/cable classes can be found in the `motors and cables`_ section. Each builder takes a number of parameters which are specified using the “parameters” keyword. All parameters are optional (if they are not specified they will take on default values). Some of the most common parameters for tgRodInfo and tgBasicActuatorInfo are shown in the example above. More information about the parameters used by tgRodInfo_ or tgBasicActuatorInfo_ can be found in their respective classes.


Adding Single-Node Structures versus Multiple-Element Structures (Spheres vs. Rods/Boxes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Most stuctures are built out of more than one node. For example, rods are made from two nodes: one for each endpoint of the rod. Consequently, to make a rod, you'd need to specify two nodes (with whatever tag you'd like), then make a pair with a *specific* tag which then must correlate with a builder. For example, let's emphasize using a full example of making one rod:

::

  nodes:
    rodEndA: [-5, 0, 0]
    rodEndB: [5, 0, 0]

  pair_groups:
    rod:
      - [rodEndA, rodEndB]

  builders:
    rod:
      class: tgRodInfo
      parameters:
        density: 0.688
        radius: 0.31

Here, you can see that "rodEndA" and "rodEndB" are arbitrary names. You could have chosen whatever you'd like, as long as the same node names (tags) are used for the line in pair_groups.
Instead, the tag "rod" is the important one! That's what correlates the nodes to a specific type of builder.


This is NOT the case when creating single-element rigid bodies. For example, consider a sphere. Spheres only have one node associated with them: their centerpoint. Since it doesn't make sense to have spheres be a "pair," NTRTsim implements the following.
Single-element rigid bodies must correlate the *node tag* to a builder, not the *pair tag* to a builder.
Here's an example of creating a sphere.

::

  nodes:
    examplesphere: [0, 5, 0]

  builders:
    examplesphere:
      class: tgSphereInfo
      parameters:
        density: 0.5
        radius: 2


If you create a pair_group with the same tag as used for a builder, nothing will happen. (To-do: check and confirm nothing is accidentally created, handle these edge cases.)
Point is - use the above example for making spheres.

Combining Structures
-----------------------------------------

Shown below is an example of a spine-like structure, made by combining six Tetrahedrons.
::

  substructures:
    t1/t2/t3/t4/t5/t6:
      path: ../BaseStructures/Tetrahedron.yaml
      offset: [0, 0 , -12]

  bond_groups:
    string:
      t1/t2/t3/t4/t5/t6/node_node:
        - [front, front]
        - [right, right]
        - [back, back]
        - [left, left]
        - [right, front]
        - [right, left]
        - [back, front]
        - [back, left]

Multiple substructures can be combined in a superstructure using the “substructures” keyword. Superstructures inherit builders defined in substructures. If there is an overlap between builders in a superstructure and a substructure, the builder in the superstructure will override the builder in the substructure.

Child Structure Attributes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each substructure is defined by a name and one or more attributes. If multiple structures share the same attribute value, an abbreviated syntax (as shown above) using the slash character can be used.

Path
'''''''''''''''''''''''''''''''''''''''''

Every child structure must be provided with a file path. The path can be absolute or relative to the parent structure.

Rotation
'''''''''''''''''''''''''''''''''''''''''

Rotation attributes are always applied first regardless of the order in which they are defined. Rotation attributes are defined as shown below using an axis, angle and an optional reference point. The axis (a vector array) refers to the axis of rotation, the angle (in degrees) refers to the angle that the structure is rotated by and the reference point (a coordinate array) refers to the point around which the structure is rotated. If no reference point is specified the center of the structure is used as a reference point.
::

  example_structure:
    rotation:
      axis: [1, 0, 0]
      angle: 90
      reference: [0, 0, 0]

Scale
'''''''''''''''''''''''''''''''''''''''''

The scale attribute scales the child structure by a specified amount. This value must be a decimal number, not a fraction. Structures are scaled around their center.
::

  example_structure:
    scale: 0.5

Translation
'''''''''''''''''''''''''''''''''''''''''

The translation attribute moves the structure by the specified vector array.
::

  example_structure:
    translation: [0, 20, 0]

Offset
'''''''''''''''''''''''''''''''''''''''''

The offset attribute is useful for spine-like structures where multiple structures are added in a row and individual structures need to be offset from one another. Each structure is offset from its preceding structure by the specified offset vector.
::

  t1/t2/t3/t4/t5/t6:
    offset: [0, 0 , -12]

Connections Between Structures
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Connections between structures can be defined using the “bond_groups” keyword. Similar to “pair_groups”, “bond_groups” are grouped by tags. Bonds are defined by the name of the structures that are being connected and the bond type used to connect those structures. In the example below the “top” node from the foot is connected to the “bottom” node of the leg using a node-to-node connection.
::

  bond_groups:
    string:
      foot/leg/node_node:
        - [top, bottom]


If the “leg” structure is itself a superstructure with multiple substructures (such as a “knee” substructure) then the child node can be specified using the dot notation shown below. The dot character is used to denote a child structure. Multiple dot characters can be used if necessary (eg. “parent.child.grandchild.node”). Using this notation is only necessary if the node name is not unique among a structure’s children.
::

  bond_groups:
    string:
      foot/leg/node_node:
        - [top, knee.bottom]

Series of Substructures (Including Spine Structures)
'''''''''''''''''''''''''''''''''''''''''

A series of substructures, for example the vertebrae in a spine structure, can be easily defined using the syntax below. The syntax makes it possible to define a set of pairs that is used to connect more than two structures. When using this syntax with slashes between multiple substructures, the connections are only made between adjacent substructures in the series of slashes. In this example, the front-to-front connection is made between t1 and t2 (since they are next to each other) as well as t2 to t3 (for the same reason), but no connection is made from t1 to t3.
::

  bond_groups:
    string:
      t1/t2/t3/node_node:
        - [front, front]

Node-to-Node Connections
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Node-to-node connections simply add a string between two specified nodes (similar to how “pair_groups” work). For superstructures it is recommended to use the bond_groups syntax rather than the “pair_groups” syntax because it makes it clear which structures are being connected (without having to use the dot notation to repeat which structure the nodes for each pair belong to).

Node-to-Edge Connections
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The structure below shows six 3-Prisms combined using node-to-edge connections.
::

  substructures:
    3prism1/3prism2/3prism3/3prism4/3prism5/3prism6:
      path: ../BaseStructures/3Prism.yaml

  bond_groups:
    horizontal_string:
      3prism1/3prism2/3prism3/3prism4/3prism5/3prism6/node_edge:
        - [top1, bottom1/bottom2]
        - [top2, bottom2/bottom3]
        - [top3, bottom3/bottom1]

Node-to-edge connections are defined using three or more pairs, where each pair consists of a node and an edge. Nodes can be connected to edges and vice versa. There can even be a mix of node-to-edges and edges-to-nodes within a single “node_edge” connection. An edge is defined by two nodes separated by the slash character. Node-to-edge connections work by attaching a node directly to the middle of a string. For node-to-edge connections, rotations and translation attributes for child structures are done automatically by the YAML model builder and do not need to be specified.

.. _motors and cables: motors-and-cables.html
.. _tgRodInfo: http://ntrt.perryb.ca/doxygen/classtg_rod_info.html
.. _tgBasicActuatorInfo: http://ntrt.perryb.ca/doxygen/classtg_basic_actuator_info.html

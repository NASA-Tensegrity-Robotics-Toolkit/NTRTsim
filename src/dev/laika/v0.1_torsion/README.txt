To run the simulation of Laika with torsion via cables (not the rotating vertebra, just pulling on the saddle cables):

From the corresponding build/ folder, after compiling, run the app with the TetrahedralSpineDropped.yaml model.
For example, the command from Drew's terminal looks like:

./AppLaikaTorsion ../../../../src/dev/laika/v0.1_torsion/TetrahedralSpineDropped.yaml 

So when typed in, the terminal looks like:
drew@bestlabserver:~/repositories/NTRTsim/build/dev/laika/v0.1_torsion$ ./AppLaikaTorsion ../../../../src/dev/laika/v0.1_torsion/TetrahedralSpineDropped.yaml

Explanation:
We are currently keeping the model files (.yaml) in the src/ directory and not copying them over to the build/ directories when the .cpp code compiles. We haven't gotten around to a better way of doing this. So, you've got to use all those ../../ to back out of the current directory.
In other words, an equivalent command would be the following, again using Drew's user:

./AppLaikaTorsion /home/drew/repositories/NTRTsim/src/dev/laika/v0.1_torsion/TetrahedralSpineDropped.yaml

or using ~ as the shortcut for the home directory:
./AppLaikaTorsion ~/repositories/NTRTsim/src/dev/laika/v0.1_torsion/TetrahedralSpineDropped.yaml


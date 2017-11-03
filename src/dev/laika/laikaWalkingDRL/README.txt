To run the simulation of Laika with a bending spine:
From the corresponding build/ folder, after compiling, run the app with the TetrahedralSpineDropped.yaml model.
For example, the command from Drew's terminal looks like:

./AppLaikaBending ../../../../src/dev/laika/v0.1_bending/TetrahedralSpineDropped.yaml 

So when typed in, the terminal looks like:
drew@bestlabserver:~/repositories/NTRTsim/build/dev/laika/v0.1_bending$ ./AppLaikaBending ../../../../src/dev/laika/v0.1_bending/TetrahedralSpineDropped.yaml

Explanation:
We are currently keeping the model files (.yaml) in the src/ directory and not copying them over to the build/ directories when the .cpp code compiles. We haven't gotten around to a better way of doing this. So, you've got to use all those ../../ to back out of the current directory.
In other words, an equivalent command would be the following, again using Drew's user:

./AppLaikaBending /home/drew/repositories/NTRTsim/src/dev/laika/v0.1_bending/TetrahedralSpineDropped.yaml

or using ~ as the shortcut for the home directory:
./AppLaikaBending ~/repositories/NTRTsim/src/dev/laika/v0.1_bending/TetrahedralSpineDropped.yaml


Setup/Installation
==================

Installing NTRT Without a Virtual Machine
-----------------------------------------
See this page for more information:

https://raw.githubusercontent.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim/master/INSTALL

Installing NTRT in a Virtual Machine
------------------------------------

Currently the NTRT OS X build does not work, and Windows is not supported. If you use either operating system you will need to run NTRT in VirtualBox (or some other virtualization software) in the meantime. Here's how you do that:

For now, here is how to install it and get it going...
 
1) Install VirtualBox on your computer. https://www.virtualbox.org/. Theoretically you could use whatever virtualization program you wanted, but virtualbox is easy to learn.
2) Download the .ova file: http://ntrt.perryb.ca/storage/vm/ntrt_vb_1-1.ova
3) Open Virtualbox. "File -> Import Appliance" and select the .ova file.
4) Adjust the number of processors, RAM, and video memory you'd like to allocate to the guest. I'd suggest 2 processors, 4GB of RAM, and max out the video memory. If your computer is a bit slower, you might be able to get away with one one processor and down to 2 or 3GB of RAM.
5) Once it's done importing, click on "Settings" to confirm things are OK. If you see an error in the bottom of the Settings window that says "Invalid Settings Detected," change your settings according to the error's recommendation.
6) Check the "Enable 3D Acceleration" box under "Display." This makes the simulation visualization run at a reasonable speed.
7) Run the virtual machine, and enjoy! You should be able to run the SUPERball demo by opening up a Terminal window and typing the commands:
8) Once your virtual machine boots up, you'll be prompted for a password the "tensegribuntu" user. The password is tensegrity.

Once the virtual machine is online, you can test it by opening a terminal and running the following:
 
cd /home/tensegribuntu/NTRTsim/bin
./build.sh
cd /home/tensegribuntu/NTRTsim/build/examples/SUPERball
./AppSuperBall
 
Known Issues:
- Virtualbox's LibGL 3D drivers are wonky and throw an error when they're loaded: it's actually fine and working though. https://www.virtualbox.org/ticket/12941 (Links to an external site.)
- The screen resolution options don't work from within Xubuntu: just drag/resize the window and it should automatically change the resolution, though.

Once you have NTRT installed and running, open your web browser and it will open three NTRT related pages in separate tabs: tutorials, doxygen and NTRT's github page. The tutorials are a good place to start in gaining familiarity with the sytstem, or in order to learn how to contribute changes you make into the NTRT master branch.

General Linux Help
------------------

In case it helps, here is a brief list of commands you can run in the terminal: http://www.pixelbeat.org/cmdline.html (Links to an external site.). I know Linux is not intuitive at first, but I'm here to help!

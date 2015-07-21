# On-Off-Keying-module


// This is a guide, or moreso, a list or resources giving instructions on how to install NS-3 and run a simulation

LINKS:

Overall NS-3 tutorial: https://www.nsnam.org/docs/release/3.21/tutorial/singlehtml/index.html
Installation guide: https://www.nsnam.org/wiki/Installation
                -Follow the instructions for installation depending on which flavor of Linux

Once an example script is made and compiles, to run this script it must be in the scratch directory.
                                          -To get to this directory run the cmd: cd repos/ns-3-allinone/ns-3-dev/scratch
                                          
NS-3 file documentation: https://www.nsnam.org/doxygen/index.html

Description:
                Currently this module mimcs that of a VLC connection whose base mimcs a point-to-point connection.We have added a couple of different modulation schemes to work with and these can be seen in the error models. The main module files are in the model folder and the helper is there to make seting objects of this type in a script easier and require less overhead. 
                The Scratch folder included, has a couple of example scripts that show how the module objects are used in a simulation. These scripts try to show the use of various facets of this module such s throughput calculations and values for BER and SNR of the signal. This is a work in progress and the people working on it exclusively will be changing throughout the duration of this project.


//installing the actual module

Prerequisites:

-Having the NS3 environment installed and running in your linux environment

-ability to open .zip files

Installation:

1) download .zip file in this repository: https://github.com/RyanAckerman/On-Off-Keying-module

2) open .zip file and you should see a simple folder named On-Off-Keying-module-master

3) enter that folder and you should see folders that mimic the ones in the repository

4) Now, go into the terminal and build a skeleton module by typing "./create-module.py on-off-keying-module"

5) Find your NS3 file system and in the "src" folder there should be an on-off-keying-module folder, open that and you should see similar folder names to that of the repository.

6) you should delete those files as they will be replaced by the files downloaded from the repository

7) Look back to where you unziped the downloaded repository files and copy and paste those folders and files into this newly made folder (DO NOT COPY THE SCRATCH FILE OVER AS IT IS NOT NEEDED IN THE MODULE FOLDER)

8) Now, start up the terminal and go to the ns-3-dev directory

9) While in this directory build all the NS3 modules (if unsure use the installation guide to help, I use ubuntu and my command is ./waf build)

10) It is optional to enable tests and examples but it should build regardless

11) It may take a while but it should build, and now you can use the on-off-keying-module in NS3 simulations

12) as far as the error model and propagation model are concerned, they can be also put into the model folder of the module but propagation can also be added onto the propagation module that is native to ns3, just be sure to add everything to its respective wscript


//Running a script

1) create a file a .cc file and follow the format that the example scripts use (in terms of laying out the script)

2) move the file into the /scratch directory

3) to compile and run use the command "./waf --run scratch/<filename>"

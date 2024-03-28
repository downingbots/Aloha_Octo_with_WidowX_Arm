# USING ALOHA'S OCTO MODEL WITH A LOW-END ROBOT ARM

In my repository "DeepMind_RT_X_with_WidowX_Arm", I tried to provide
a bare-bones implementation to run the DeepMind RT-X Models with a low-end robot arm. Unfortunately the RT-X model did not seem generalize to a "previous generation" variation of the BridgeData hardware configuration. My next step is to gather a small dataset to fine-tune the Octo model (which also is trained using a subset of the RT-X dataset) and see if the arm has a much higher success rate at performing language instructions.

The Octo model is pretrained for the Aloha robot ( https://www.trossenrobotics.com/aloha.aspx ) The Octo model supports fine-tuning to control your own robot arm.  The repository for the Octo model is:
https://github.com/octo-models/octo

Unfortunately, my WidowX arm is pretty old and its Arbotix microprocessor had a flakey Serial Port that eventually went kaput. The Arbotix microproccessor is no longer supported or available, so I replaced it with the Robotis U2D2 and the equivelent of a U2D2 Power Hub Board.  I migrated the C++ code on the Arbotix microprocessor to Python for the laptop.  Now 100% python, the laptop directly controls the robot arm.  I also added a simple joystick controller and tweaked the rt1_widowx.py script to run with the new python version.  

This python version of the code is now checked in to this repository as a starting point to playing with fine-tuning with the Octo model.


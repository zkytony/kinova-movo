# movo_v1
MOVO repository for the Kinova mobile manipulator

Remote PC and sim does not need movo_network or movo_robot.

Setup Instructions: https://github.com/Kinovarobotics/kinova-movo/wiki/Setup-Instructions

Note: voice control requires installation of pocketsphinx. eg: sudo apt-get install ros-kinetic-pocketsphinx
    voice navigation requires installation of SpeechRecognition. eg: pip install SpeechRecognition

# Installation guide
## kinect-devel
* Follow the steps in movo_common/si_utils/src/si_utils/setup_movo_pc_migration. Start from Install third parties and additionnal libraries. But do it line by line manually.

* In the above steps, make sure you use gcc-5. When doing `cmake`, do `env CXX=g++-5 cmake` instead.

* For libfreenect2, follow the instruction given by Kinova <https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions>.

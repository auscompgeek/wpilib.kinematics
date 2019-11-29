Installation
------------

For local development on a computer with an internet connection:

.. code-block:: bash

   pip install wpilib.kinematics

To install onto an offline roboRIO:

.. code-block:: bash

   # with an internet connection
   robotpy-installer download-pip wpilib.kinematics
   # whilst connected to your robot
   robotpy-installer install-pip wpilib.kinematics

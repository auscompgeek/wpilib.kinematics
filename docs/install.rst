Installation
------------

For local development on a computer with an internet connection:

.. warning::

   It is anticipated that RobotPy WPILib will have these classes in the 2020.2.2 update.
   This package will conflict with such an update, and *must* be uninstalled before updating.

.. code-block:: bash

   pip install wpilib.kinematics

To install onto an offline roboRIO:

.. code-block:: bash

   # with an internet connection
   robotpy-installer download-pip wpilib.kinematics
   # whilst connected to your robot
   robotpy-installer install-pip wpilib.kinematics

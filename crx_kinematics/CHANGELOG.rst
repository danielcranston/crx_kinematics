^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package crx_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-02-21)
------------------
* Implement FK/IK for the Fanuc CRX series cobots, following closely `Geometric Approach for Inverse Kinematics of the FANUC CRX Collaborative Robot by Abbes and Poisson (2024) <https://hal.science/hal-04915402v1/file/IFToMM2024-02-20.pdf>`_.
* Add support for all robots in the CRX series
* Add MoveIt 2 Kinematics plugin
* Add tests for FK/IK and MoveIt Kinematics plugin
* Add CI that builds and tests for Humble, Jazzy, Kilted and Rolling
* Contributors: Daniel Cranston

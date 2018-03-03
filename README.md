# Motion Planning and Control
Overview of the subsystem:

![Motion Planning and Control Overview](Motion_Planning_and_Control_ROS_Stack.png)

## unicycle_controller

This node serves as the high-level controller for path following. It follows a non-linear control law based on unicycle vehicle dynamics, which our modified Firebird 0xDelta robot follows.

![Unicycle Controller Example](unicycle_controller/unicycle_controller.png)

Built from scratch (Based on C. Canudas de Wit., Nonlinear Control Design for Mobile Robots, in The International Journal of Robotics Research.)

_Status: Yet to tune controller gains but working well otherwise_

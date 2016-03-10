% Connect gripper for running the control algorithms.

gripper = robotiq;
gripper.Connect();
gripper.ChangeMode('Individual');
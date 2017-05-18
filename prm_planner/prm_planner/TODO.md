# TODO
- Absturz Planer nach Action Server
- Crash Pouring: Absturz bei Aufruf Jacobian
test - Geschwindigkeit Roboter anpassen
- Collision Checks bei Grasping werden dauerhaft deaktiviert
- Fallbacklösung für fehlende Transformation Objekt->Gripper
- Multiarm Grasping/Dropping
- Transformation zwischen Objekt und Greifer fix lassen und nicht updaten, wenn das Objekt sich im Greifer befindet.
- Änderung der Posen bei Setzen von constraints nicht immer sinnvoll (trinken!!!)
- Überprüfung des Pouring Planers
- Handtreiber verbessern
- update prm aufrufen
- waitForData von robot arm sollte auch die Hand Zustände prüfen
- ist initRobotControl im controller noch nötig?
- entfernen der alten packages
- Objekt im Greifer bei der Kollisionserkennung berücksichtigen
- Einschenken debuggen
- Aktivieren/Deaktivieren des Planners sorgt für TF Probleme

# Papers
## Optimization
- Increasing Efficiency of Optimization-based Path Planning for Robotic Manipulators
Hao Ding, Gunther Reißig, Olaf Stursberg
http://ieeexplore.ieee.org/ielx5/6149620/6159299/06161276.pdf?tp=&arnumber=6161276&isnumber=6159299
- Using Mixed Integer Programs to find a path using optimization. 
Quite slow, even for primitive robots in known environments. No real-world experiments. 
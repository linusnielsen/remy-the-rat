My attempt at improving the Remy rat originally conceived by [@shebuildsrobots](https://www.instagram.com/shebuildsrobots/). Her project is here: https://github.com/shebuildsrobots/remy_model. It was later improved by these guys at Hackaday: https://hackaday.io/project/202363-remy-the-rat.

My goal was to make Remy's arms move in sync with mine, giving the impression that Remy is controlling me. Or rather, my wife, since I don't have hair. :)

I took the Blender file from the Hackaday project and exported the complete rat to FreeCAD, and carved out a profile that fit the sub-micro servos.
I then stole the Mahony filter function from https://github.com/jremington/MPU-6050-Fusion to get a completely stable yaw/pitch/roll from the MPU6050.

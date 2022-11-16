# FRC Team 3512's 2022 Offseason Robot

Source code also for the 2022 offseason robot: Xiphos (but with swerve)

## Game

The game for 2022 is called Rapid React, where teams are tasked with shooting cargo into either a lower or upper hub. This year there is the standard 15 second autonomous period. Teams earn points in this period for moving off of the tarmac and earn double the usual points for scoring in the hub. If the alliance can score 5 cargo during autonomous, then the number of cargo needed for the Tele-op ranking point is reduced from 20 to 18. Tele-op continues this gameplay while the robot is controlled through driver input and the cargo points are cut in half. Alliances can earn a ranking point for scoring a certain number of cargo in the hub, be it upper or lower. The number of cargo is dependent on how much cargo was scored during autonomous as stated previously. In Tele-op there is the addition to climb in the hangar. The hangar has four levels of bars the increase in height from the ground and are separated from each other. They cascade over and up towards the driverstation. Robots can traverse up these bars, gaining more points for the higher bar they reach, gaining enough points as an alliance grants a ranking point. Climbing is not prohibited to a certain time during the match, meaning there is no Endgame sequence.

## Unique features

This years robot's unique features include:

- Swerve drive (MK4 L4 modules using CTRE CANCoders and NEO motors)
- Fourbar linkage intake/outtake
- Funnel
- Indexing conveyor
- Front and Back Flywheels run independently for variable shooting distances
- Raspberry Pi 3 w/ vision processing
- Passive arms that leave robot hanging on bar
- Active climber arms that can be actuated forward and back to traverse to highest rung

## Goals of the year
|Status|Goal|Additional Description|
|------|----|----------------------|
|Yes|Swerve Drive|Working swerve drive code using a derivative of Team 364's BaseFalconSwerve|
|Yes|Separate Flywheel State Space Controllers|Use separate state spaces controllers to run a front and back flywheel and different speeds for variable shooting.|
|Yes|AdvantageKit|Move onto the AdvantageKit logging framework and use AdvantageScope to view the data.|
|No|PathPlanner|Implement PathPlanner and a working auton using PathPlanner|
|No|Pose Estimation|Implement pose estimation on the swerve drive using the updated SwerveDrivePoseEstimator from the 2023 beta.|

## Roster

Students: Adan Silva (Lead), Jonathan Shuman, Grady Sargeant

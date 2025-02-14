# Rough Elevator Documentation and Notes
by motormaniac

# TODO:
Implement **Control Flow** for the elevator motors:
[text](https://docs.revrobotics.com/revlib/spark/closed-loop)

### General Notes
- Some of the inputs in the autologged inputs class have arrays because it tracks multiple motors. We want to log the temp and current in all of them.
    - In advantage scope, this shows up as a drop down menu

### Elevator Specific Notes
- Elevator uses 2 motors (1 is inverted)
- Use a `SparkClosedLoopController` because its easier and has built in ff
- Use `elevatorsim` not `singlejointedarmsim`
    - I think this is a revlib class that integrates with wpi


> [!WARNING]
> Use the resetmode in `sparkmax`, **NOT** the resetmode in `servo


**I used velocityFF for elevator pid. Ask if this is a problem?**

# RoboBee Adaptive Controller

This folder contains the Simulink-generated code for the RoboBee adaptive controller. 

## Files

- `integrated_controller.h` - Main header file for the controller
- `integrated_controller.c` - Implementation of the controller
- `integrated_controller_data.c` - Controller data and parameters
- `rtwtypes.h` - Simulink runtime types definition

## Integration

This controller is integrated into the EntoBench framework through the `AdaptiveController` wrapper class defined in `src/ento-control/adaptive_controller.h`. The wrapper provides a clean interface that matches the pattern used by other controllers in the codebase.

## Future Work

In a future iteration, this Simulink-generated code should be replaced with a native implementation that is more maintainable and better integrated with the rest of the EntoBench framework. 
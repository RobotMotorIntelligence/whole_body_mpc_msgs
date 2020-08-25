# This message describes the Riccati controller input.
#
# A Riccati controller implements feedback state gains together with feedforward commands.
# To be able to compute feedback commands, it is required to define a reference whole-state.
# A Riccati gains is used to apply state feedback control.
#
# The header specifies the time at which the state is published.


Header header

whole_body_state_msgs/WholeBodyState[] reference
RiccatiGain[] gain


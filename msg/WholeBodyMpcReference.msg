# This message describes the whole-body MPC controller output.
#
# A whole-body MPC controller implements feedback state gains together with whole-body references.
# To be able to compute feedback commands, it is required to convert the centroidal information into a base one.
# The feedforward torque commands are stored inside the whole-body reference, i.e. in joint effor field.
#
# The header specifies the time at which the state is published.


Header header

whole_body_state_msgs/WholeBodyState[] reference
StateFeedbackGain[] gain


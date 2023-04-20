# Legged Body Planner Msg
The Legged Body Msg incorporates a generic msg interface s.t. any traditional motion planning algorithm can be interpolated into a rigid body plan

## Remarks
An example of the generality of this message interface
- Suppose a 2D motion plan containing $x \in R^2$
- States of plan contains $x, \dot{x}$ with control $u$
- Can use the State.msg to contain a 2D plan and then later convert the State.msg to a Rigid Body Model plan

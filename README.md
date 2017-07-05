# House Boat Simulator

A simple simulation of a 15' x 47' two-level houseboat with a 140 HP motor, similar to [this one](https://lakeshasta.com/houseboats/the-empress/).

## Instructions

Run the simulator in any modern browser by visiting https://dkirkby.github.io/hb

There are two controls, steering (displayed along the bottom edge) and throttle (displayed along the right edge), which can be adjusted several ways:

Steering:
 - keyboard: left / right arrow turns wheel left / right.
 - mouse: click on left / right of circle turns wheel left /right.
 - touch screen: touch left / right of circle turns wheel left /right.

Throttle:
 - keyboard: up / down arrow pushes throttle up / down.
 - mouse: click above / below circle pushes throttle up / down.
 - touch screen: touch above / below circle pushes throttle up / down.

Hold down the key / mouse / finger to continue adjusting the steering or throttle.

Try not to collide with the screen edges or the docks.  If you do, the corners of your boat will appear increasingly red to indicate the amount of damage to your boat.

For a challenge, try to bring your boat to a full stop inside the mooring space in under 120 seconds (elapsed time appears in the upper-left corner).  For more of a challenge, navigate outside the L shape and park stern first in the mooring space.

## Physics Model

The boat is modeled as a rigid body subject to the following forces on its center of mass:
 - propeller thrust.
 - anisotropic drag (larger for sideways motion).
 - collisions with the edges and docks.

Each of these forces also generates a corresponding torque on the boat.  Collisions are model using non-linear springs that repel a boat corner penetrating a fixed edge or a fixed corner penetrating a boat edge.

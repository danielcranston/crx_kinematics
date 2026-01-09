# Fanuc pose configurations

Recall that a 6DOF cartesian pose can map to several joint poses. In other words, there is a one-to-many relation between a cartesian pose and joint poses.

On the Fanuc controller, the datatype for cartesian poses is imbued with extra information (a **configuration**) that enables the pose to be uniquely specifiable, allowing a one-to-one mapping (a bijection) between cartesian poses and joint poses.

This configuration holds 3 booleans and 3 integers (called "Turn Numbers"). When looking at a cartesian pose on the pendant, its configuration is visible as a string after `CONF:` on the top-right. An example of such a string is `NUT 000`. Here's a nice summary: https://www.youtube.com/watch?v=QVRqgc2ak4k

| Configuration Element  | Description | Possible values |
| ------------- | ------------- | ------------- |
| Boolean 1 | Flip the wrist? I.e., invert the sign of $`J5`$? |  `N` (**N**ot flipped) / `F` (**F**lipped) |
| Boolean 2 | Elbow up ($`J2 < J3`$ **¹** ) or down? | `U` (**U**p) / `D` (**D**own)  |
| Boolean 3 | J1 forwards ($` \lVert J1 \rVert < 90°`$ **²** ) or backwards? | `T` (Fron**t**) / `B` (**B**ack)  |
| Turn Number 1 | J1 below, within, or beyond ±180°? | `-1` / `0` / `1`  |
| Turn Number 2 | J4 below, within, or beyond ±180°? | `-1` / `0` / `1` |
| Turn Number 3 | J6 below, within, or beyond ±180°? | `-1` / `0` / `1` |

**¹** : Here, `J3` is the "coupled" (controller) version, not the "uncoupled" (official URDF) one. See [DERIVATION.md](DERIVATION.md).\
**²** : Slight simplification that assumes a neutral first Turn Number. "J1 forwards" is probably more expressive.

**Disclaimer**: This reflects my understanding, which might be wrong. I'm not aware of any Fanuc documents that can veryify all these details.

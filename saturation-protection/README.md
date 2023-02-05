# Adaptive Control in the Presence of Input Constraints

This repository contains a simple adaptive control example, implemented in the [Python Control Systems Library](https://python-control.org/).
The example was taken from work by S.P. Karason and A.M. Annaswamy.
Two manuscripts, both titled _Adaptive Control in the Presence of Input Constraints_ can be found [here](https://doi.org/10.23919/ACC.1993.4793095) and [here](https://doi.org/10.1109/9.333787).
A small writeup on the controller implemented in this code is available at [http://danielwiese.com/posts/adaptive-control-input-constraints/](http://danielwiese.com/posts/adaptive-control-input-constraints/).

The plots below show the results of two simulations - in both cases an adaptive controller is attempting to drive the plant state `x_p` to the reference state `x_m` with control effort limited to `U_MAX = 10`.
In the first plot, no saturation protection is used.
This is accomplished by setting `dot_beta_delta = 0` and `dot_e_delta = 0` in `adaptive_state()`.
In the second plot, saturation protection adaptive controller is used.

<p align="center">
  <img src="https://github.com/dpwiese/control-examples/blob/main/saturation-protection/fig/no_saturation_protection_state_input.png?raw=true" width="600">
</p>

<p align="center">
  <img src="https://github.com/dpwiese/control-examples/blob/main/saturation-protection/fig/saturation_protection_state_input.png?raw=true" width="600">
</p>

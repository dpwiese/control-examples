# Sigma Modification

This repository contains a simple adaptive control example using sigma modification implemented in the [Python Control Systems Library](https://python-control.org/).
In the presence of a constant input bias, the "standard" adaptive law will cause the parameter estimate to grow unbounded.
The use of sigma modification shown here prevents this from happening, although it doesn't provide convergence of the tracking error to zero.

<p align="center">
  <img src="https://github.com/dpwiese/control-examples/blob/main/sigma-mod/fig/sigma_mod.png?raw=true" width="600">
</p>

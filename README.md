# Control Systems with Python

This repository contains some adaptive control examples using the [Python Control Systems Library](https://github.com/python-control/python-control).
The blog post [Adaptive PI Control with Python](https://danielwiese.com/posts/adaptive-pi-python/) walks through an example and might be helpful.
To get started check out the Python Control Systems Library [documentation](https://python-control.readthedocs.io/en/0.8.3/index.html).

# Installation

The following installation instructions were written for OSX, but should be easily adapted to other environments.

## Dependencies

From the docs:

> The package requires numpy and scipy, and the plotting routines require matplotlib. In addition, some routines require the slycot library in order to implement more advanced features

Start by installing the basic dependencies

```sh
pip3 install numpy scipy matplotlib
```

To install the [Slycot](https://github.com/python-control/Slycot) first install some dependencies.

### gfortran

First a FORTRAN compiler is needed.
The post [How to install gfortran on Mac OS X](https://skipperkongen.dk/2012/04/27/how-to-install-gfortran-on-mac-os-x/) was helpful pointing to [Sourceforge](http://hpc.sourceforge.net) to download gfortran for OSX and extract the archive with the `tar`:

```sh
sudo tar -xvf gcc-9.2-bin.tar -C /
```
### CMake

Next CMake was downloaded from [https://cmake.org/download/](https://cmake.org/download/), installed, and to the shell path `.bashrc` or `.zshrc`:

```sh
PATH="/Applications/CMake.app/Contents/bin":"$PATH"
```

### Slycot

A first attempt to install [Slycot](https://github.com/python-control/Slycot) with the following command

```sh
pip3 install slycot
```

resulted in the error:

```
AttributeError: module 'enum' has no attribute 'IntFlag'?
```

The following Stack Overflow post [Why Python 3.6.1 throws AttributeError: module 'enum' has no attribute 'IntFlag'?](https://stackoverflow.com/questions/43124775/why-python-3-6-1-throws-attributeerror-module-enum-has-no-attribute-intflag) provides the following solution:

```sh
pip3 uninstall -y enum34
```

A second attempt to install Slycot resulted in another error:

```
AttributeError: type object 'Callable' has no attribute '_abc_registry'
```

Another Stack Overflow post [AttributeError: type object 'Callable' has no attribute 'abc_registry'](https://stackoverflow.com/questions/55833509/attributeerror-type-object-callable-has-no-attribute-abc-registry) gave the following solution:

```sh
pip3 uninstall typing
```

Finally Slycot was installed successfully.
Yet another error was encountered when attempting to import Slycot and yet another Stack Overflow post, [How to fix ImportError: cannot import name wrapper using slycot](https://stackoverflow.com/questions/33002887/how-to-fix-importerror-cannot-import-name-wrapper-using-slycot) offered a solution: `brew install gcc49` which unfortunately did not work:

```
gcc@4.9: This formula either does not compile or function as expected on macOS
versions newer than High Sierra due to an upstream incompatibility.
```

Discussion in a relatively recent issue [Slycot/issues/46](https://github.com/python-control/Slycot/issues/46) suggested instead of installing gcc, to instead rename the wrapper in `site-packages/slycot` directory, the location of which was easily found with `python3 -v` followed by `import slycot` and on my environment was in:

```
/Library/Frameworks/Python.framework/Versions/3.7/lib/python3.7/site-packages/slycot
```

The next attempt to import Slycot resulted in the error:

```
ModuleNotFoundError: No module named 'numpy.core._multiarray_umath'
```

Which was finally solved by a quick reinstallation of numpy.

```sh
pip3 uninstall numpy
pip3 install numpy
```

Finally all the dependencies were installed and working correctly.

## Python Control Systems Library

The Python Control Systems Library was then easily installed:

```sh
pip3 install control
```

# Using Python Control

Some examples and commonly used references:

https://python-control.readthedocs.io/en/0.8.3/cruise-control.html
https://python-control.readthedocs.io/en/0.8.3/generated/control.iosys.NonlinearIOSystem.html
https://python-control.readthedocs.io/en/0.8.3/generated/control.iosys.LinearIOSystem.html
https://python-control.readthedocs.io/en/0.8.3/generated/control.iosys.InterconnectedSystem.html
https://python-control.readthedocs.io/en/0.8.3/conventions.html#lti-system-representation

```python
num = np.array([1.])
den = np.array([J, B])
sys = control.TransferFunction(num, den)

sys = StateSpace(A, B, C, D)

s = TransferFunction.s
G  = (s + 1)/(s**2 + 2*s + 1)
```

Note: https://stackoverflow.com/questions/6392739/what-does-the-at-symbol-do-in-python

> If you see an @ in the middle of a line, that's a different thing, matrix multiplication.

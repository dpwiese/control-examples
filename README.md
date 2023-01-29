# Control Systems with Python

This repository contains some adaptive control examples using the [Python Control Systems Library](https://github.com/python-control/python-control).
The blog post [Adaptive PI Control with Python](https://danielwiese.com/posts/adaptive-pi-python/) walks through an example and might be helpful.
To get started check out the Python Control Systems Library [documentation](https://python-control.readthedocs.io/en/0.8.3/index.html).

## Using

Switch to Python 3.8 before running the code.

```sh
pyenv local 3.8.16
```

This is tested with Python Control `0.9.3.post2`.

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
The post [How to install gfortran on Mac OS X](https://skipperkongen.dk/2012/04/27/how-to-install-gfortran-on-mac-os-x/) was helpful pointing to [Sourceforge](http://hpc.sourceforge.net) to download gfortran for OSX and extract the archive with the `tar` command:

```sh
gunzip gcc-12.1-m1-bin.tar.gz
sudo tar -xvf gcc-12.1-m1-bin.tar -C /
```

Now we can check that it was installed

```sh
gfortran --version

# GNU Fortran (GCC) 12.1.0
# Copyright (C) 2022 Free Software Foundation, Inc.
# This is free software; see the source for copying conditions. There is NO
# warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

### CMake

Next CMake was downloaded from [https://cmake.org/download/](https://cmake.org/download/), installed, and to the shell path `.bashrc` or `.zshrc`:

```sh
PATH="/Applications/CMake.app/Contents/bin":"$PATH"
```

### Slycot

A first attempt to install [Slycot](https://github.com/python-control/Slycot) with the following command failed.

```sh
pip3 install slycot
```

The following section describes the couple errors I encountered during installation and the steps to solve the errors.

#### Installation Errors

The following errors were the resulting of following the same steps to install Slycot on a different environment (M1 Macbook, MacOS Monterey).

```
ERROR: Could not build wheels for scipy which use PEP 517 and cannot be installed directly
```

First upgrade pip.

```sh
/Library/Developer/CommandLineTools/usr/bin/python3 -m pip install --upgrade pip
```

Next installed [Ninja](https://ninja-build.org/) with

```sh
brew install ninja
```

Next error

```
Could NOT find Python (missing: Python_LIBRARIES Interpreter Development
NumPy Development.Module Development.Embed)
```

Couldn't quite figure this one out after a few minutes of searching, but saw a comment that using Python 3.8 worked.
Install Python 3.8 and switch using

```sh
pyenv install 3.8
pyenv local 3.8.16
```

Afterwards everything worked as expected.

## Python Control Systems Library

The Python Control Systems Library was then easily installed:

```sh
pip3 install control
```

#### Installation Errors on Previous Python Version

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

# Using Python Control

Some examples and commonly used references:

* [Cruise control design example (as a nonlinear I/O system)](https://python-control.readthedocs.io/en/0.9.3.post2/cruise-control.html)
* [control.NonlinearIOSystem](https://python-control.readthedocs.io/en/0.9.3.post2/generated/control.NonlinearIOSystem.html)
* [control.LinearIOSystem](https://python-control.readthedocs.io/en/0.9.3.post2/generated/control.LinearIOSystem.html)
* [control.InterconnectedSystem](https://python-control.readthedocs.io/en/0.9.3.post2/generated/control.InterconnectedSystem.html)
* [LTI system representation](https://python-control.readthedocs.io/en/0.9.3.post2/conventions.html#lti-system-representation)

```python
num = np.array([1.])
den = np.array([J, B])
sys = control.TransferFunction(num, den)

sys = StateSpace(A, B, C, D)

s = TransferFunction.s
G  = (s + 1)/(s**2 + 2*s + 1)
```

Note: [What does the "at" (@) symbol do in Python?](https://stackoverflow.com/questions/6392739/what-does-the-at-symbol-do-in-python)

> If you see an @ in the middle of a line, that's a different thing, matrix multiplication.

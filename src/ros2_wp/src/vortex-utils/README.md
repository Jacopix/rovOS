# Introduction
[![Industrial CI](https://github.com/vortexntnu/vortex-utils/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-utils/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/vortex-utils/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/vortex-utils/main)
[![codecov](https://codecov.io/github/vortexntnu/vortex-utils/graph/badge.svg?token=d6D7d5xNdf)](https://codecov.io/github/vortexntnu/vortex-utils)

This repository contains general, often-used utility functions and structs for both C++ and Python.


# Usage


In Python, import your desired function/dataclass like for example:
```python
from vortex_utils.python_utils import ssa
```

In C++, include vortex_utils like
```C++
#include <vortex_utils/cpp_utils.hpp>
```
and in the code
```C++
double some_angle = 3.14;
double angle_ssa = vortex_utils::ssa(some_angle);
```

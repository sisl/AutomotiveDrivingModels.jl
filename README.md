# Automotive Driving Models

A Julia package containing tools for simulating automotive driving models

[![Build Status](https://travis-ci.org/sisl/AutomotiveDrivingModels.jl.svg?branch=master)](https://travis-ci.org/sisl/AutomotiveDrivingModels.jl)
[![Coverage Status](https://coveralls.io/repos/github/sisl/AutomotiveDrivingModels.jl/badge.svg?branch=master)](https://coveralls.io/github/sisl/AutomotiveDrivingModels.jl?branch=master)
[![](https://img.shields.io/badge/docs-latest-blue.svg)](https://sisl.github.io/AutomotiveDrivingModels.jl/latest)


Example for [1D driving](http://nbviewer.jupyter.org/github/sisl/AutomotiveDrivingModels.jl/blob/master/docs/1DMobius.ipynb)

For visualization code please see [AutoViz](https://github.com/sisl/AutoViz.jl).

## Installation 

Using the SISL registry (recommanded): 

```julia 
] registry add https://github.com/sisl/Registry
] add AutomotiveDrivingModels 
```

Manually installing all the dependencies:

```julia 
using Pkg
Pkg.add(PackageSpec(url="https://github.com/sisl/Vec.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/Records.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveDrivingModels.jl"))
```

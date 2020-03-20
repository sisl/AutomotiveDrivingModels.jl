# Vec 

`Vec` is a submodule of AutomotiveDrivingModels that provides several vector types, named after their groups. All types are immutable and are subtypes of ['StaticArrays'](https://github.com/JuliaArrays/StaticArrays.jl)' `FieldVector`, so they can be indexed and used as vectors in many contexts.

* `VecE2` provides an (x,y) type of the Euclidean-2 group.
* `VecE3` provides an (x,y,z) type of the Euclidean-3 group.
* `VecSE2` provides an (x,y,theta) type of the special-Euclidean 2 group.

```julia
v = VecE2(0, 1)
v = VecSE2(0,1,0.5)
v = VecE3(0, 1, 2)
```

Additional geometry types include `Quat` for quaternions, `Line`, `LineSegment`, and `Projectile`.

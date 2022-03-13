## 2D Gemoetry module

### Polygon/polyline

The two classes provide a wrapper to the CGAL data structures. You may need to add the following compile configuration to your module if depending on the two classes:

```
target_compile_options(your_module PUBLIC "-frounding-math")
```

A workaround for this requirement.
```
target_compile_definitions(geometry PUBLIC "-DCGAL_DISABLE_ROUNDING_MATH_CHECK")
```

Reference:
* [1] https://github.com/cms-sw/cmssw/issues/14862
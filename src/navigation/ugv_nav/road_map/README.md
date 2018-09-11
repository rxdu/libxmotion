## Additional requirements for the map file (.osm)

* for any lanelet, you have to additionally assign a name of type "name" (right below "type" attribute)
* a relation of type "reference", containing a node of role "origin".
* a relation of type "center_line", containing a series of nodes of role "point" and a path of role "line". The name should be "cl_<lanelet-name>"

Example:

```
<relation id='-39216' action='modify'>
    <member type='node' ref='-39134' role='origin' />
    <tag k='type' v='reference' />
</relation>
```
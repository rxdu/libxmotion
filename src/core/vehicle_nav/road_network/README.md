## Additional requirements for the map file (.osm)

* a relation of type "reference", containing a node of role "origin".
* a relation of type "reference", containing a series of nodes of role "refline". The boundary points must be specified in order clockwise.

Example:

```
<relation id='-39216' action='modify'>
    <member type='node' ref='-39134' role='origin' />
    <tag k='type' v='reference' />
</relation>

<relation id='-39871' action='modify'>
    <member type='node' ref='-39124' role='refline' />
    <member type='node' ref='-39122' role='refline' />
    ...
    <tag k='type' v='reference' />
  </relation>
```
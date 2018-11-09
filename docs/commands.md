# Commands

* Create gif animation using "imagemagick"

```
$ convert -delay 120 -loop 0 *.png animated.gif
```

* Get statistics of the code base

```
$ sudo apt install cloc
$ cd ~/Workspace/librav/src
$ cloc --exclude-dir=lcmtypes,third_party .
```
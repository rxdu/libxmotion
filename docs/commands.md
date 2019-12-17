# Commands

## Create gif animation using "imagemagick"

```
$ convert -delay 120 -loop 0 *.png animated.gif
```

## Create gif from mp4

```
sudo apt-get install ffmpeg
wget -O opengl-rotating-triangle.mp4 https://github.com/cirosantilli/media/blob/master/opengl-rotating-triangle.mp4?raw=true
ulimit -Sv 1000000
ffmpeg \
  -i opengl-rotating-triangle.mp4 \
  -r 15 \
  -vf scale=512:-1 \
  -ss 00:00:03 -to 00:00:06 \
  opengl-rotating-triangle.gif
```

## Get statistics of the code base

```
$ sudo apt install cloc
$ cd ~/Workspace/librav/src
$ cloc --exclude-dir=cmake,lcmtypes,third_party .
```

## git subtree

Adding the sub-project as a remote
```
$ git remote add -f [remote-name] [remote-url]
$ git subtree add --prefix [sub-project-name] [remote-name] [branch-name] --squash
```

Update the sub-project
```
$ git fetch [remote-name] [branch-name] 
$ git subtree pull --prefix [sub-project-name] [remote-name] [branch-name] --squash
```

Push to remote
```
$ git subtree push --prefix=[sub-project-name] [remote-name] [branch-name] 
```

Firmware branch update
```
$ git fetch fw_origin pios_pixcar
$ git subtree pull --prefix firmware fw_origin pios_pixcar --squash
```

## Reference

* [Git subtree: the alternative to Git submodule](https://www.atlassian.com/blog/git/alternatives-to-git-submodule-git-subtree)
* [Gif from mp4](https://askubuntu.com/questions/648603/how-to-create-an-animated-gif-from-mp4-video-via-command-line)
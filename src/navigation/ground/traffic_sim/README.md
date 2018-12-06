* Note on installation of SUMO

Install Fox dependency for SUMO-GUI

```
$ cd <your-workspace>
$ wget ftp://ftp.fox-toolkit.org/pub/fox-1.6.57.tar.gz
$ tar -xvzf fox-1.6.57.tar.gz
$ cd fox-1.6.57
$ ./configure
$ make -j8
$ sudo make install
```

Download and compile SUMO

```
$ cd <your-workspace>
$ git clone --recursive https://github.com/eclipse/sumo
$ cd sumo
$ git fetch origin refs/replace/*:refs/replace/*
$ pwd
# add the path you got from the above command to your .bashrc
export SUMO_HOME="<the-path-you-got-from-pwm-cmd>"
$ mkdir -p build/cmake-build
$ cd build/cmake-build
$ cmake ../..
$ make -j8
```

After compile finished, you can find binaries in sumo/bin folder.
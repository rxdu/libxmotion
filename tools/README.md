## Utils

This repository contains a collection of software that are used to develop and test robot related algorithms with physics-based simulation at SRCL.

### 1. Build g3log

First build the library from source

```
$ cd {path-to-srcl_ctrl}/utils/library_source/g3log_srcl
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
```

Then you can copy the compiled static library file into {path-to-srcl_ctrl}/utils/logger for easy access.

### [Reference]

* https://gehrcke.de/2011/06/reading-files-in-c-using-ifstream-dealing-correctly-with-badbit-failbit-eofbit-and-perror/
* http://stackoverflow.com/questions/7868936/read-file-line-by-line
* http://stackoverflow.com/questions/18274582/parsing-a-huge-complicated-csv-file-using-c
* http://stackoverflow.com/questions/16329358/remove-spaces-from-a-string-in-c
* http://stackoverflow.com/questions/1508939/qt-layout-on-qmainwindow
* http://doc.qt.io/qt-4.8/modelview.html
* https://gist.github.com/jmk/3906292
* http://stackoverflow.com/questions/2666367/how-to-find-item-selected-from-customcontextmenurequested-on-qtreeview-item

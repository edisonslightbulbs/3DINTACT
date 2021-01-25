#### dbscan

Dbscan (Density-Based Spatial Clustering of Applications with Noise) is a data clustering algorithm proposed by [Martin Ester, Hans-Peter Kriegel, Jörg Sander and Xiaowei Xu in 1996](https://en.wikipedia.org/wiki/Dbscan).

Epsilon estimation |  dbscan using estimated hyper-parameter
:-------------------------:|:-------------------------:
![](https://raw.githubusercontent.com/edisonslightbulbs/dbscan-cpp/main/resources/demo_.png)  |  ![](https://raw.githubusercontent.com/antiqueeverett/dbscan-cpp/main/resources/demo.png)

##### Usage

from the project root directory

```bash
mkdir build && cd build || return
cmake ..
make
```

`CMakeLists.txt` is configured to auto-run the tartget post build (i.e., nothing else is required post build).

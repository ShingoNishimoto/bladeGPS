# bladeGPS

Very crude experimental implimentation of [gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim) for real-time signal generation.
The code works with bladeRF and has been tested on Windows only.

```
Usage: bladegps [options]
Options:
  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)
  -u <user_motion> User motion file (dynamic mode)
  -g <nmea_gga>    NMEA GGA stream (dynamic mode)
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 35.274,137.014,100"
  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss
  -d <duration>    Duration [sec] (max: 86400)
```

### Additional include files and libraries

1. libbladeRF.h and bladeRF.lib (build from the [source](https://github.com/Nuand/bladeRF))
2. pthread.h and pthreadVC2.lib (available from [sourceware.org/pthreads-win32](https://sourceware.org/pthreads-win32/))

### Build on Linux

1. Retrive the bladeRF source in a directory next to the current directory.

 ```
$ cd ..
$ git clone git@github.com:Nuand/bladeRF.git
```

2. Build the bladeRF host library.

 ```
$ cd bladeRF/host
$ mkdir build
$ cd build
$ cmake ..
$ make
```

3. Build bladeGPS.

 ```
$ cd ../../../bladeGPS
$ make
```

### License

Copyright &copy; 2015 Takuji Ebinuma  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).

# Motor-Propeller-Matching
 A Python script for evaluating efficiency alignment of a motor/propeller combination.


To properly enter propeller data, you must choose a file with the following example format:
```
J       CT       CP       eta
0.165   0.0993   0.0539   0.304
0.214   0.0947   0.0543   0.374
0.255   0.0916   0.0548   0.427
etc...
```
If pulling from the UIUC database, note that you will ususally have to combine a couple files
to get the whole advance ratio range.

For Automated Versions (2.X.X+):
The file must be named what you want the propeller name to be, followed by a space (spaces in name are OK), followed by
the diameter in inches, followed by an 'x', followed by the pitch in inches, followed by .txt or .dat.
Like so: ```[Propeller Name] [Diameter]x[Pitch].txt```

file must look like:
```
J       CT       CP       eta
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
```

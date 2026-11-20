# Vehicle sizing study (flexblade.vehicle)

864 configs evaluated (0 errors), 800 feasible (battery current <= rated, motor current, end-of-discharge pulse headroom >= 0.3 of mean torque, back-EMF <= 80 % of loaded supply, all structural checks <= 1).

Rotor: TPU + CF-rod flexure (`tpu_rod`), PA6-CF blade, elastic axis 0.43c, AM32 slew 2550/s, 5-8 in, 2 blades. Motor kv rewound so hover back-EMF = 57 % of mid-SoC loaded voltage (2307-class, 2806.5-class above 150 W shaft/motor). Pulse amplitude capped at 1.0 x mean torque. `ctrl_ratio` = hub moment/(W R) at the capped amplitude and mid-SoC voltage; `ctrl@end` is scaled to the end-of-discharge voltage; `typ` is at 0.3 x mean torque.

## Best battery per (layout, size, payload) -- max endurance, feasible

| layout | D in | payload g | battery | AUW g | pack g | kv | hover W | endur min | endur min (BEM -23 %) | I hover A | C | ctrl@end | ctrl typ |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| single | 5 | 0 | liion_21700 6S1P | 622 | 482 | 1500 | 172 | 31.9 | 22.1 | 7.8 | 1.6 | 0.0031 | 0.0009 |
| single | 5 | 50 | liion_21700 6S1P | 672 | 482 | 1560 | 192 | 28.5 | 19.9 | 8.8 | 1.7 | 0.0030 | 0.0009 |
| single | 5 | 100 | liion_21700 6S1P | 722 | 482 | 1620 | 212 | 25.7 | 17.8 | 9.7 | 1.9 | 0.0028 | 0.0008 |
| single | 5 | 200 | liion_21700 6S1P | 848 | 482 | 1740 | 250 | 21.7 | 14.7 | 11.5 | 2.3 | 0.0015 | 0.0004 |
| single | 6 | 0 | liion_21700 6S1P | 645 | 482 | 1060 | 143 | 38.7 | 26.8 | 6.5 | 1.3 | 0.0125 | 0.0037 |
| single | 6 | 50 | liion_21700 6S1P | 695 | 482 | 1100 | 159 | 34.7 | 23.9 | 7.2 | 1.4 | 0.0116 | 0.0035 |
| single | 6 | 100 | liion_21700 6S2P | 1247 | 958 | 1440 | 346 | 31.6 | 21.0 | 15.8 | 1.6 | 0.0036 | 0.0011 |
| single | 6 | 200 | liion_21700 6S2P | 1347 | 958 | 1500 | 392 | 27.9 | 18.4 | 17.9 | 1.8 | 0.0033 | 0.0010 |
| single | 7 | 0 | liion_21700 6S2P | 1174 | 958 | 1030 | 264 | 41.8 | 27.8 | 12.0 | 1.2 | 0.0165 | 0.0049 |
| single | 7 | 50 | liion_21700 6S2P | 1224 | 958 | 1050 | 282 | 39.1 | 26.0 | 12.8 | 1.3 | 0.0156 | 0.0047 |
| single | 7 | 100 | liion_21700 6S2P | 1274 | 958 | 1070 | 300 | 36.7 | 24.3 | 13.6 | 1.4 | 0.0148 | 0.0044 |
| single | 7 | 200 | liion_21700 6S2P | 1374 | 958 | 1110 | 338 | 32.4 | 21.4 | 15.4 | 1.5 | 0.0134 | 0.0040 |
| single | 8 | 0 | liion_21700 6S2P | 1203 | 958 | 790 | 241 | 45.9 | 30.4 | 10.9 | 1.1 | 0.0704 | 0.0211 |
| single | 8 | 50 | liion_21700 6S2P | 1253 | 958 | 810 | 257 | 43.0 | 28.4 | 11.6 | 1.2 | 0.0792 | 0.0238 |
| single | 8 | 100 | liion_21700 6S2P | 1303 | 958 | 830 | 274 | 40.3 | 26.6 | 12.4 | 1.2 | 0.0900 | 0.0270 |
| single | 8 | 200 | liion_21700 4S2P | 1086 | 641 | 1150 | 218 | 33.5 | 22.0 | 14.9 | 1.5 | 0.0640 | 0.0192 |
| coaxial | 5 | 0 | liion_21700 6S2P | 1180 | 958 | 1620 | 410 | 26.6 | 18.2 | 18.8 | 1.9 | 0.0029 | 0.0009 |
| coaxial | 5 | 50 | liion_21700 6S2P | 1282 | 958 | 1680 | 436 | 24.9 | 17.1 | 20.1 | 2.0 | 0.0016 | 0.0005 |
| coaxial | 5 | 100 | liion_21700 6S2P | 1332 | 958 | 1710 | 460 | 23.6 | 16.1 | 21.2 | 2.1 | 0.0016 | 0.0005 |
| coaxial | 5 | 200 | liion_21700 6S2P | 1432 | 958 | 1780 | 511 | 21.1 | 14.4 | 23.7 | 2.3 | 0.0015 | 0.0004 |
| coaxial | 6 | 0 | liion_21700 6S2P | 1217 | 958 | 1130 | 337 | 32.6 | 22.1 | 15.4 | 1.5 | 0.0121 | 0.0036 |
| coaxial | 6 | 50 | liion_21700 6S2P | 1267 | 958 | 1160 | 358 | 30.6 | 20.7 | 16.3 | 1.6 | 0.0116 | 0.0035 |
| coaxial | 6 | 100 | liion_21700 6S2P | 1317 | 958 | 1180 | 378 | 28.9 | 19.5 | 17.3 | 1.7 | 0.0112 | 0.0034 |
| coaxial | 6 | 200 | liion_21700 6S2P | 1469 | 958 | 1240 | 410 | 26.6 | 18.0 | 18.8 | 1.9 | 0.0055 | 0.0016 |
| coaxial | 7 | 0 | liion_21700 6S1P | 784 | 482 | 690 | 154 | 35.7 | 24.6 | 7.0 | 1.4 | 0.0407 | 0.0122 |
| coaxial | 7 | 50 | liion_21700 4S2P | 993 | 641 | 1150 | 223 | 32.8 | 22.2 | 15.2 | 1.5 | 0.0735 | 0.0221 |
| coaxial | 7 | 100 | liion_21700 4S2P | 1043 | 641 | 1170 | 239 | 30.5 | 20.5 | 16.4 | 1.6 | 0.0860 | 0.0258 |
| coaxial | 7 | 200 | liion_21700 4S2P | 1143 | 641 | 1230 | 275 | 26.4 | 17.6 | 19.0 | 1.9 | 0.1231 | 0.0369 |
| coaxial | 8 | 0 | liion_21700 6S2P | 1308 | 958 | 660 | 291 | 37.8 | 25.1 | 13.2 | 1.3 | 0.0182 | 0.0073 |
| coaxial | 8 | 50 | liion_21700 6S2P | 1358 | 958 | 670 | 309 | 35.6 | 23.6 | 14.0 | 1.4 | 0.0173 | 0.0075 |
| coaxial | 8 | 100 | liion_21700 6S2P | 1408 | 958 | 680 | 326 | 33.7 | 22.2 | 14.8 | 1.5 | 0.0163 | 0.0077 |
| coaxial | 8 | 200 | liion_21700 6S2P | 1508 | 958 | 710 | 363 | 30.2 | 19.8 | 16.6 | 1.6 | 0.0161 | 0.0081 |

## Pareto: endurance vs payload (feasible, any size/battery, per layout)

| layout | payload g | endur min | D in | battery | AUW g | ctrl@end |
|---|---|---|---|---|---|---|
| single | 0 | 45.9 | 8 | liion_21700 6S2P | 1203 | 0.0704 |
| single | 50 | 43.0 | 8 | liion_21700 6S2P | 1253 | 0.0792 |
| single | 100 | 40.3 | 8 | liion_21700 6S2P | 1303 | 0.0900 |
| single | 200 | 33.5 | 8 | liion_21700 4S2P | 1086 | 0.0640 |
| coaxial | 0 | 37.8 | 8 | liion_21700 6S2P | 1308 | 0.0182 |
| coaxial | 50 | 35.6 | 8 | liion_21700 6S2P | 1358 | 0.0173 |
| coaxial | 100 | 33.7 | 8 | liion_21700 6S2P | 1408 | 0.0163 |
| coaxial | 200 | 30.2 | 8 | liion_21700 6S2P | 1508 | 0.0161 |

## Pareto: control_ratio@end vs endurance -- single, payload 0 g

| endur min | ctrl@end | ctrl typ | D in | battery | AUW g | kv | hover W |
|---|---|---|---|---|---|---|---|
| 45.9 | 0.0704 | 0.0211 | 8 | liion_21700 6S2P | 1203 | 790 | 241 |
| 41.9 | 0.0738 | 0.0221 | 8 | liion_18650 6S3P | 1224 | 800 | 248 |
| 26.6 | 0.0749 | 0.0225 | 7 | lipo 6S 2.2Ah | 554 | 700 | 98 |
| 25.8 | 0.0792 | 0.0238 | 7 | lipo 4S 3.3Ah | 554 | 1050 | 101 |

## Pareto: control_ratio@end vs endurance -- single, payload 100 g

| endur min | ctrl@end | ctrl typ | D in | battery | AUW g | kv | hover W |
|---|---|---|---|---|---|---|---|
| 40.3 | 0.0900 | 0.0270 | 8 | liion_21700 6S2P | 1303 | 830 | 274 |
| 31.1 | 0.1172 | 0.0352 | 7 | liion_21700 4S1P | 613 | 1150 | 118 |
| 21.7 | 0.1311 | 0.0393 | 7 | lihv 6S 2.2Ah | 645 | 740 | 122 |

## Pareto: control_ratio@end vs endurance -- coaxial, payload 0 g

| endur min | ctrl@end | ctrl typ | D in | battery | AUW g | kv | hover W |
|---|---|---|---|---|---|---|---|
| 37.8 | 0.0182 | 0.0073 | 8 | liion_21700 6S2P | 1308 | 660 | 291 |
| 35.7 | 0.0407 | 0.0122 | 7 | liion_21700 6S1P | 784 | 690 | 154 |
| 35.5 | 0.0635 | 0.0190 | 7 | liion_21700 4S2P | 943 | 1120 | 206 |
| 32.7 | 0.0660 | 0.0198 | 7 | liion_18650 4S3P | 956 | 1120 | 211 |
| 14.1 | 0.0742 | 0.0223 | 6 | lihv 4S 1.5Ah | 424 | 1040 | 85 |
| 11.7 | 0.1169 | 0.0351 | 6 | lihv 3S 1.5Ah | 384 | 1340 | 77 |
| 8.6 | 0.1225 | 0.0367 | 6 | lipo 3S 1.0Ah | 347 | 1310 | 68 |

## Pareto: control_ratio@end vs endurance -- coaxial, payload 100 g

| endur min | ctrl@end | ctrl typ | D in | battery | AUW g | kv | hover W |
|---|---|---|---|---|---|---|---|
| 33.7 | 0.0163 | 0.0077 | 8 | liion_21700 6S2P | 1408 | 680 | 326 |
| 31.1 | 0.0186 | 0.0056 | 8 | liion_21700 6S1P | 932 | 570 | 176 |
| 30.5 | 0.0860 | 0.0258 | 7 | liion_21700 4S2P | 1043 | 1170 | 239 |
| 28.1 | 0.0899 | 0.0270 | 7 | liion_18650 4S3P | 1056 | 1170 | 244 |

## Why configs fail (count of failed flags among all evaluated)

- structure_ok: 64
- headroom_end_ok: 10

## Coaxial vs single at the best battery (same D, payload)

| D | payload g | single min | coax min | coax/single | single ctrl@end | coax ctrl@end |
|---|---|---|---|---|---|---|
| 5 | 0 | 31.9 | 26.6 | 0.83 | 0.0031 | 0.0029 |
| 5 | 50 | 28.5 | 24.9 | 0.87 | 0.0030 | 0.0016 |
| 5 | 100 | 25.7 | 23.6 | 0.92 | 0.0028 | 0.0016 |
| 5 | 200 | 21.7 | 21.1 | 0.98 | 0.0015 | 0.0015 |
| 6 | 0 | 38.7 | 32.6 | 0.84 | 0.0125 | 0.0121 |
| 6 | 50 | 34.7 | 30.6 | 0.88 | 0.0116 | 0.0116 |
| 6 | 100 | 31.6 | 28.9 | 0.91 | 0.0036 | 0.0112 |
| 6 | 200 | 27.9 | 26.6 | 0.95 | 0.0033 | 0.0055 |
| 7 | 0 | 41.8 | 35.7 | 0.85 | 0.0165 | 0.0407 |
| 7 | 50 | 39.1 | 32.8 | 0.84 | 0.0156 | 0.0735 |
| 7 | 100 | 36.7 | 30.5 | 0.83 | 0.0148 | 0.0860 |
| 7 | 200 | 32.4 | 26.4 | 0.81 | 0.0134 | 0.1231 |
| 8 | 0 | 45.9 | 37.8 | 0.82 | 0.0704 | 0.0182 |
| 8 | 50 | 43.0 | 35.6 | 0.83 | 0.0792 | 0.0173 |
| 8 | 100 | 40.3 | 33.7 | 0.83 | 0.0900 | 0.0163 |
| 8 | 200 | 33.5 | 30.2 | 0.90 | 0.0640 | 0.0161 |


#
#
 
M
o
d
e
l
 
a
n
d
 
c
a
v
e
a
t
s
 
(
r
e
a
d
 
b
e
f
o
r
e
 
t
r
u
s
t
i
n
g
 
a
n
y
 
n
u
m
b
e
r
)




-
 
*
*
A
e
r
o
*
*
:
 
B
E
M
 
h
o
v
e
r
 
t
h
r
u
s
t
 
i
s
 
p
r
o
b
a
b
l
y
 
~
2
3
 
%
 
o
p
t
i
m
i
s
t
i
c
 
(
f
i
r
s
t
 
c
o
a
r
s
e
 
C
F
D
,
 
s
e
e


 
 
`
s
t
u
d
y
/
c
f
d
_
v
a
l
i
d
a
t
i
o
n
.
m
d
`
)
.
 
 
T
h
e
 
"
B
E
M
 
-
2
3
 
%
"
 
e
n
d
u
r
a
n
c
e
 
c
o
l
u
m
n
 
r
e
-
t
r
i
m
s
 
w
i
t
h
 
t
h
e
 
r
e
q
u
i
r
e
d
 
t
h
r
u
s
t


 
 
i
n
f
l
a
t
e
d
 
t
o
 
W
/
0
.
7
7
 
(
r
p
m
 
u
p
,
 
p
o
w
e
r
 
~
+
4
0
 
%
)
;
 
t
h
e
 
f
i
r
s
t
 
c
o
l
u
m
n
 
i
s
 
t
h
e
 
r
a
w
 
B
E
M
.
 
 
T
r
e
a
t
 
r
e
a
l
i
t
y
 
a
s


 
 
n
e
a
r
e
r
 
t
h
e
 
d
e
r
a
t
e
d
 
c
o
l
u
m
n
.
 
 
C
o
a
x
i
a
l
 
i
n
t
e
r
f
e
r
e
n
c
e
 
i
s
 
t
h
e
 
a
c
t
u
a
t
o
r
-
d
i
s
k
 
m
o
d
e
l
 
i
n
 
`
c
o
a
x
i
a
l
.
p
y
`
.


-
 
*
*
B
a
t
t
e
r
y
 
d
a
t
a
 
a
r
e
 
t
y
p
i
c
a
l
,
 
n
o
t
 
m
e
a
s
u
r
e
d
*
*
:
 
M
o
l
i
c
e
l
 
P
4
5
B
 
4
.
5
 
A
h
/
6
9
 
g
/
4
5
 
A
/
1
3
 
m
O
h
m
 
(
r
e
t
a
i
l
e
r
 
s
h
e
e
t
s
,


 
 
n
o
t
 
t
h
e
 
o
f
f
i
c
i
a
l
 
d
a
t
a
s
h
e
e
t
)
,
 
P
5
0
B
 
6
0
 
A
 
u
n
v
e
r
i
f
i
e
d
,
 
1
8
6
5
0
 
a
n
d
 
L
i
P
o
/
L
i
H
V
 
v
a
l
u
e
s
 
a
s
s
u
m
e
d
 
f
r
o
m
 
t
y
p
i
c
a
l


 
 
F
P
V
 
p
a
c
k
s
 
(
1
5
0
 
W
h
/
k
g
 
c
e
l
l
,
 
1
0
 
%
 
w
i
r
i
n
g
 
o
v
e
r
h
e
a
d
;
 
L
i
-
i
o
n
 
1
5
 
%
 
f
o
r
 
h
o
l
d
e
r
s
/
s
t
r
i
p
s
)
.
 
 
C
o
n
s
t
a
n
t
-
p
o
w
e
r


 
 
d
i
s
c
h
a
r
g
e
 
w
i
t
h
 
O
C
V
 
t
a
b
l
e
 
a
n
d
 
s
e
r
i
e
s
 
R
;
 
n
o
 
P
e
u
k
e
r
t
/
t
e
m
p
e
r
a
t
u
r
e
/
a
g
e
i
n
g
.
 
 
U
s
a
b
l
e
 
f
r
a
c
t
i
o
n
s
 
0
.
8
5


 
 
(
L
i
P
o
)
 
/
 
0
.
9
2
 
(
L
i
-
i
o
n
)
 
i
n
c
l
u
d
e
 
n
o
 
l
a
n
d
i
n
g
 
r
e
s
e
r
v
e
 
b
e
y
o
n
d
 
t
h
a
t
.


-
 
*
*
M
a
s
s
 
b
u
d
g
e
t
*
*
 
(
f
r
a
m
e
 
6
0
 
g
 
(
D
/
5
)
^
1
.
5
,
 
x
1
.
5
 
c
o
a
x
;
 
m
o
t
o
r
s
 
3
3
/
5
5
 
g
;
 
E
S
C
 
1
0
-
1
4
 
g
;
 
a
v
i
o
n
i
c
s
 
2
8
 
g
;


 
 
h
u
b
 
6
 
g
 
(
D
/
5
)
^
2
;
 
b
l
a
d
e
s
/
f
l
e
x
u
r
e
 
f
r
o
m
 
g
e
o
m
e
t
r
y
 
x
 
P
A
6
-
C
F
/
T
P
U
 
d
e
n
s
i
t
y
)
 
i
s
 
+
-
3
0
 
%
;
 
f
r
a
m
e
 
i
s
 
t
h
e


 
 
l
a
r
g
e
s
t
 
g
u
e
s
s
.
 
 
A
U
W
 
i
s
 
a
 
s
t
r
a
i
g
h
t
 
s
u
m
 
-
-
 
t
h
e
 
s
w
e
e
p
 
d
o
e
s
 
n
o
t
 
c
o
n
s
t
r
a
i
n
 
A
U
W
 
o
r
 
s
i
z
e
 
t
h
e
 
r
o
t
o
r
 
t
o


 
 
t
h
e
 
p
a
c
k
,
 
s
o
 
"
b
e
s
t
 
e
n
d
u
r
a
n
c
e
"
 
t
e
n
d
s
 
t
o
 
p
i
c
k
 
t
h
e
 
l
a
r
g
e
s
t
 
p
a
c
k
;
 
t
h
e
 
o
p
t
i
m
u
m
 
f
o
r
 
a
 
r
e
a
l
 
b
u
i
l
d
 
i
s
 
a


 
 
m
a
x
-
A
U
W
 
o
r
 
m
a
x
-
p
o
w
e
r
-
l
i
m
i
t
 
q
u
e
s
t
i
o
n
,
 
n
o
t
 
s
h
o
w
n
 
h
e
r
e
.


-
 
*
*
M
o
t
o
r
*
*
:
 
k
v
 
r
e
w
o
u
n
d
 
p
e
r
 
s
u
p
p
l
y
 
(
R
_
e
f
f
 
=
 
R
_
f
i
x
e
d
 
+
 
R
_
w
i
n
d
 
(
k
v
_
r
e
f
/
k
v
)
^
2
,
 
m
a
x
 
c
u
r
r
e
n
t
 
4
0
 
A
 
/


 
 
5
0
 
A
)
 
s
o
 
h
o
v
e
r
 
b
a
c
k
-
E
M
F
 
=
 
5
7
 
%
 
o
f
 
m
i
d
-
S
o
C
 
l
o
a
d
e
d
 
v
o
l
t
a
g
e
;
 
t
h
e
 
l
i
b
r
a
r
y
 
2
3
0
7
 
R
,
 
L
,
 
b
,
 
T
0
 
a
r
e
 
t
h
e


 
 
H
a
k
e
a
 
f
i
t
;
 
L
 
i
s
 
a
 
+
-
3
0
 
%
 
e
s
t
i
m
a
t
e
.
 
 
T
h
e
r
m
a
l
 
l
i
m
i
t
s
 
a
r
e
 
n
o
t
 
m
o
d
e
l
l
e
d
.


-
 
*
*
c
o
n
t
r
o
l
_
r
a
t
i
o
 
i
s
 
N
O
T
 
a
n
 
o
p
t
i
m
i
s
e
d
 
f
l
e
x
u
r
e
 
r
e
s
u
l
t
.
*
*
 
 
T
h
e
 
f
l
e
x
u
r
e
 
i
s
 
t
h
e
 
s
c
a
l
e
d
 
`
t
p
u
_
r
o
d
`


 
 
p
l
a
c
e
h
o
l
d
e
r
 
a
t
 
e
a
c
h
 
s
i
z
e
/
m
a
s
s
;
 
c
o
n
t
r
o
l
 
a
u
t
h
o
r
i
t
y
 
d
e
p
e
n
d
s
 
s
t
r
o
n
g
l
y
 
o
n
 
w
h
e
r
e
 
t
h
e
 
f
l
a
p
/
l
a
g
/
p
i
t
c
h


 
 
r
e
s
o
n
a
n
c
e
s
 
l
a
n
d
 
r
e
l
a
t
i
v
e
 
t
o
 
t
h
e
 
h
o
v
e
r
 
r
p
m
 
(
n
o
t
e
 
t
h
e
 
j
u
m
p
s
 
b
e
t
w
e
e
n
 
5
-
6
 
i
n
 
a
n
d
 
8
 
i
n
)
.
 
 
R
e
-
t
u
n
e
 
t
h
e


 
 
f
l
e
x
u
r
e
 
p
e
r
 
v
e
h
i
c
l
e
 
w
i
t
h
 
`
o
p
t
i
m
i
z
e
`
 
b
e
f
o
r
e
 
c
o
m
p
a
r
i
n
g
 
l
a
y
o
u
t
s
 
o
n
 
c
o
n
t
r
o
l
_
r
a
t
i
o
.
 
 
P
u
l
s
e
 
a
m
p
l
i
t
u
d
e
 
i
s


 
 
c
a
p
p
e
d
 
a
t
 
1
.
0
 
x
 
m
e
a
n
 
t
o
r
q
u
e
;
 
a
t
 
t
h
i
s
 
c
a
p
 
t
h
e
 
p
u
l
s
e
 
s
l
e
w
/
h
u
b
-
s
p
e
e
d
 
o
s
c
i
l
l
a
t
i
o
n
 
m
a
y
 
a
l
r
e
a
d
y
 
e
x
c
e
e
d


 
 
w
h
a
t
 
t
h
e
 
E
S
C
/
h
u
b
 
t
o
l
e
r
a
t
e
s
 
(
s
l
e
w
 
u
t
i
l
 
n
o
t
 
a
 
g
a
t
e
 
h
e
r
e
)
.
 
 
"
t
y
p
"
 
=
 
0
.
3
 
x
 
m
e
a
n
 
t
o
r
q
u
e
 
(
l
i
n
e
a
r


 
 
s
c
a
l
i
n
g
)
.


-
 
E
x
t
r
a
 
c
o
p
p
e
r
 
f
r
o
m
 
c
y
c
l
i
c
 
p
u
l
s
i
n
g
 
a
t
 
t
h
e
 
t
y
p
i
c
a
l
 
0
.
3
 
a
m
p
l
i
t
u
d
e
 
i
s
 
o
n
l
y
 
~
4
.
5
 
%
 
o
f
 
m
o
t
o
r
 
c
o
p
p
e
r


 
 
(
a
 
f
e
w
 
W
)
 
-
-
 
n
e
g
l
i
g
i
b
l
e
 
a
g
a
i
n
s
t
 
h
o
v
e
r
 
p
o
w
e
r
;
 
E
S
C
 
s
w
i
t
c
h
i
n
g
 
2
 
%
 
+
 
i
d
l
e
,
 
a
v
i
o
n
i
c
s
 
1
.
5
 
W
.


-
 
F
e
a
s
i
b
i
l
i
t
y
 
f
l
a
g
s
:
 
h
o
v
e
r
 
b
a
t
t
e
r
y
 
C
 
<
=
 
r
a
t
e
d
;
 
p
e
a
k
 
(
1
.
3
 
x
 
h
o
v
e
r
)
 
<
=
 
1
.
5
 
x
 
r
a
t
e
d
;
 
m
o
t
o
r


 
 
c
u
r
r
e
n
t
;
 
e
n
d
-
o
f
-
d
i
s
c
h
a
r
g
e
 
p
u
l
s
e
 
h
e
a
d
r
o
o
m
 
>
=
 
0
.
3
;
 
b
a
c
k
-
E
M
F
 
<
=
 
8
0
 
%
 
o
f
 
l
o
a
d
e
d
 
e
n
d
 
v
o
l
t
a
g
e
;


 
 
s
t
r
u
c
t
u
r
a
l
 
c
h
e
c
k
s
 
(
s
t
a
t
i
c
,
 
c
r
e
e
p
,
 
f
a
t
i
g
u
e
,
 
i
n
t
e
r
l
a
y
e
r
)
 
<
=
 
1
.



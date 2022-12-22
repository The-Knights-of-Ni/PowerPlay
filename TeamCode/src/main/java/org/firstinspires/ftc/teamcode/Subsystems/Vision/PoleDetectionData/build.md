Format the positive images if manually generated: `python3 ./tools/mergevec.py -v ./positive/ -o ./positive.vec`

**OR**

Create the positive images with `opencv_createsamples`:

`opencv_createsamples -o positive.vec -img [image of pole] -bg [various backgrounds] `

Create the dataset for use
```
opencv_traincascade -data haar -bg ./bg.txt -numThreads 5 -vec ./positive.vec -numStages 20
-minHitRate 0.999 -maxFalseAlarmRate 0.5 -numPos [fill] -numNeg [fill] -w [fill] -h [fill]
-mode ALL -precalcValBufSize 4096 -precalcIdxBufSize 4096 -mode ALL
```
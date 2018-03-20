#!/bin/bash
mkdir ../data

wget http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
bunzip2 shape_predictor_68_face_landmarks.dat.bz2
mv shape_predictor_68_face_landmarks.dat ../data/

wget https://storage.cmusatyalab.org/openface-models/nn4.small2.v1.t7
mv nn4.small2.v1.t7 ../data/

rm shape_predictor_68_face_landmarks.dat.bz2

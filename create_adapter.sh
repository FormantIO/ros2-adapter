#!/bin/bash

mkdir adapter
cp -R formant_ros2_adapter adapter/

cp * adapter/

zip -r adapter.zip adapter

rm -rf adapter
#!/bin/bash

mkdir ~/dddmr_bags

echo -n "Do you want to download depth camera bag files (101MB)? (Y/N):"
read d_bag
if [ "$d_bag" != "${d_bag#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1bKBosqJjREsddwJh0jMrOVxGpI9pLXwb \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o multi_depth_camera_images.zip \
      'https://drive.usercontent.google.com/download?id='1bKBosqJjREsddwJh0jMrOVxGpI9pLXwb'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip multi_depth_camera_images.zip
fi
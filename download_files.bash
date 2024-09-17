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

echo -n "Do you want to download multilayer lidar (leishen c16) bag files (120MB)? (Y/N):"
read m_bag
if [ "$m_bag" != "${m_bag#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1MnvlZkYrqKZwCJ74LMuCD1zh4owUq6I- \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o multilayer_spinning_lidar_pointcloud.zip \
      'https://drive.usercontent.google.com/download?id='1MnvlZkYrqKZwCJ74LMuCD1zh4owUq6I-'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip multilayer_spinning_lidar_pointcloud.zip
fi

echo -n "Do you want to download multilayer lidar (unitree g4) bag files (25MB)? (Y/N):"
read u_bag
if [ "$u_bag" != "${u_bag#[Yy]}" ] ;then 
  echo "Download bag"
  cd ~/dddmr_bags && curl -L -c cookies.txt 'https://drive.usercontent.google.com/uc?export=download&id='1QVMqS-rCi5Q96LgwVDxc-jD5VhVjwohn- \
      | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt
  curl -L -b cookies.txt -o unitree_lidar_point_cloud.zip \
      'https://drive.usercontent.google.com/download?id='1QVMqS-rCi5Q96LgwVDxc-jD5VhVjwohn-'&confirm='$(<confirm.txt)
  rm -f confirm.txt cookies.txt
  unzip unitree_lidar_point_cloud.zip
fi

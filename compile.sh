#!/bin/bash

#make
make modules
make modules_install
cd /boot
mv *5.3.1* 5.3.1.bakup/
cd /usr/src/linux-5.3.1
make install
update-grub2

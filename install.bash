#!/bin/bash

if [ $(uname -m) == 'x86_64' ]; then
  ARCH="64"
else
  ARCH="32"
fi

ARDUINO="arduino-1.0.5"
INSTALLER="${ARDUINO}-linux${ARCH}.tgz"
INSTALLER_URL="http://arduino.googlecode.com/files/$INSTALLER"

cd ~ 
if [ ! -f $INSTALLER ]; then
  wget $INSTALLER_URL
fi
tar -xvzf $INSTALLER
sudo mv ~/$ARDUINO /usr/share/arduino

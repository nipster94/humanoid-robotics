#!/bin/bash
cd ..
DIR="dlib"
if [ -d "$DIR" ]; then
    echo "dlib is already here...." 
    cd "$DIR"
    DIR="shared_build"
    if [ -d "$DIR" ]; then
       echo "We detect shared_build directory. It is possible that the system 
             already have dlib installed and configured."
       if [ -e /usr/local/lib/libdlib.a ]; then
	  echo "dlib has already been installed and configured. Abroading installation process"
          exit 0	
       else
          echo "Installation/Configuration is still required..."        
       fi
    else
       echo "Creating shared_build directory..."
       mkdir shared_build
    fi
fi

echo "Configuring dlib...."
cd shared_build
cmake ..
cmake --build . --config Release
sudo make install 
exit 0

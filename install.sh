#!/bin/bash

# actions before build
function pre_build() {
    sudo apt-get update
}

# run install
function install() {
    echo "Installing winch ..."
    
    pre_build
    
    sudo apt-get install python-numpy python-scipy python-matplotlib ipython 
    sudo apt-get install qt4-designer
    sudo apt-get install pyqt4-dev-tools
    sudo apt-get install python-kde4 python-kde4-dbg python-kde4-dev python-kde4-doc
    sudo pip install tzlocal
    
    sudo apt-get install cutecom
 
    echo "Installing winch ... Done"
}

install
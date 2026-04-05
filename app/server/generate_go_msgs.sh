#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
# Clone the repository https://github.com/ClemensElflein/open_mower_ros in a temporary directory
OM_DIR=/tmp/open_mower_ros
DYN_DIR=/tmp/dynamic_reconfigure
git clone https://github.com/ClemensElflein/open_mower_ros $OM_DIR
git clone https://github.com/ros/dynamic_reconfigure $DYN_DIR
cd $OM_DIR && git submodule update --init --recursive
cd $SCRIPT_DIR
# Use msg-import to import the messages from the temporary directory
declare -a PACKAGES_NAME
declare -a PACKAGES_PATH

PACKAGES_NAME[0]="xbot_msgs"
PACKAGES_PATH[0]="$OM_DIR/src/lib/xbot_msgs/msg"
PACKAGES_NAME[1]="mower_msgs"
PACKAGES_PATH[1]="$OM_DIR/src/mower_msgs/msg"
PACKAGES_NAME[2]="mower_map"
PACKAGES_PATH[2]="$OM_DIR/src/mower_map/msg"

PACKAGES_NAME[3]="xbot_msgs"
PACKAGES_PATH[3]="$OM_DIR/src/lib/xbot_msgs/srv"
PACKAGES_NAME[4]="mower_msgs"
PACKAGES_PATH[4]="$OM_DIR/src/mower_msgs/srv"
PACKAGES_NAME[5]="mower_map"
PACKAGES_PATH[5]="$OM_DIR/src/mower_map/srv"

PACKAGES_NAME[6]="dynamic_reconfigure"
PACKAGES_PATH[6]="$DYN_DIR/srv"
PACKAGES_NAME[7]="dynamic_reconfigure"
PACKAGES_PATH[7]="$DYN_DIR/msg"
for pkg in ${!PACKAGES_NAME[@]}; do
  mkdir -p pkg/msgs/${PACKAGES_NAME[$pkg]}
  msgList=$(find ${PACKAGES_PATH[$pkg]} -name '*.msg')
  for file in $msgList; do
    echo "Importing message $file"
    filename=$(basename $file)
    filename="${filename%.*}"
    $GOPATH/bin/msg-import --rospackage=${PACKAGES_NAME[$pkg]} --gopackage=${PACKAGES_NAME[$pkg]} $file > pkg/msgs/${PACKAGES_NAME[$pkg]}/${filename}.go
  done
  srvList=$(find ${PACKAGES_PATH[$pkg]} -name '*.srv')
  for file in $srvList; do
    echo "Importing service $file"
    filename=$(basename $file)
    filename="${filename%.*}"
    $GOPATH/bin/srv-import --rospackage=${PACKAGES_NAME[$pkg]} --gopackage=${PACKAGES_NAME[$pkg]} $file > pkg/msgs/${PACKAGES_NAME[$pkg]}/${filename}.go
  done
done

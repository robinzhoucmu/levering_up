#!/bin/bash
# edit PREPUSH2DATA_BASE=$HOME/pushdata to your push data directory

thisFile=$_
if [ $BASH ] 
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

set_levering_up_base()
{
  # use cd and pwd to get an absolute path
  configParentDir="$(cd "$(dirname "$thisFile")/.." && pwd)"

  # different cases for software/config or software/build/config
  case "$(basename $configParentDir)" in
    "software") export LEVERING_UP_BASE=$(dirname $configParentDir);;
    "build") export LEVERING_UP_BASE=$(dirname $(dirname $configParentDir));;
    *) echo "Warning: LEVERING_UP environment file is stored in unrecognized location: $thisFile";;
  esac
  export LEVERING_UPDATA_BASE=$LEVERING_UP_BASE/../levering_updata
  export PATH=$PATH:$LEVERING_UP_BASE/software/build/bin
}

setup_levering_up()
{
  export PATH=$PATH:$LEVERING_UP_BASE/software/build/bin
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=$LEVERING_UP_BASE/software/build/lib:$LEVERING_UP_BASE/software/build/lib64:$LD_LIBRARY_PATH
  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar:$LEVERING_UP_BASE/software/build/share/java/lcmtypes_levering_up_lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$LEVERING_UP_BASE/software/build/share/java/drake.jar:$LEVERING_UP_BASE/software/build/share/java/bot2-lcmgl.jar
  export PKG_CONFIG_PATH=$LEVERING_UP_BASE/software/build/lib/pkgconfig:$LEVERING_UP_BASE/software/build/lib64/pkgconfig:$PKG_CONFIG_PATH

  # python path
  export PYTHONPATH=$PYTHONPATH:$LEVERING_UP_BASE/software/build/lib/python2.7/site-packages:$LEVERING_UP_BASE/software/build/lib/python2.7/dist-packages
  # enable some warnings by default
  export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
  export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"
  
  export PATH=$PATH:$HOME/software/ffmpeg-2.4.2-64bit-static # for ffmpeg software
  
  export ROSLAUNCH_SSH_UNKNOWN=1
  
  #set main git user
  gituser jiaji
}

set_ros()
{
  if [ -f $LEVERING_UP_BASE/catkin_ws/devel/setup.bash ]; then
    source $LEVERING_UP_BASE/catkin_ws/devel/setup.bash
  else
    source /opt/ros/indigo/setup.bash
  fi
  export ROS_PACKAGE_PATH=$HOME/levering_up/ros_ws/:$ROS_PACKAGE_PATH
}

# some useful commands
alias cdlevering_up='cd $LEVERING_UP_BASE'
alias cdlevering_updata='cd $LEVERING_UPDATA_BASE'
alias matlabdrake='cd $LEVERING_UP_BASE/software; matlab -r "addpath_pods; addpath_drake"'
alias matlablevering_up='cd $LEVERING_UP_BASE/software; matlab -nodesktop -nodisplay -nosplash -r "tic; addpath_pods; addpath_drake; toc; cd ../software/planning/ik_server/; ikTrajServerSocket;"'

alias gitsub='git submodule update --init --recursive'
alias gitpull='git -C $LEVERING_UP_BASE pull'

alias rebash='source ~/.bashrc'
alias open='gnome-open'

alias yolo='rosservice call /robot1_SetSpeed 1600 180'
alias faster='rosservice call /robot1_SetSpeed 200 50'
alias fast='rosservice call /robot1_SetSpeed 100 30'
alias slow='rosservice call /robot1_SetSpeed 50 15'

alias gohome='rosservice call robot2_SetJoints "{j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0}"'

alias teleop='rosrun teleop teleop'
alias pythonlevering_up='ipython -i -c "run $LEVERING_UP_BASE/catkin_ws/src/levering_up_config/python/pythonlevering_up.py"'

alias pman='bot-procman-sheriff -l $LEVERING_UP_BASE/software/config/levering_up.pmd'

alias roslocal='export ROS_MASTER_URI=http://localhost:11311'

alias getjoint='rosservice call robot1_GetJoints'
alias getcart='rosservice call robot1_GetCartesian'
alias setjoint='rosservice call robot1_SetJoints'
alias setcart='rosservice call robot1_SetCartesian'

alias lcmlocal='sudo ifconfig lo multicast; sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo'

ppms2mp4()
{
  bot-ppmsgz $1 mpeg4 10M 30 $1.mp4
}

function lowersuffix {
  cd "$1"
  find . -name '*.*' -exec sh -c '
  a=$(echo {} | sed -r "s/([^.]*)\$/\L\1/");
  [ "$a" != "{}" ] && mv "{}" "$a" ' \;
}

function ipmasq {
   if [ $# -eq 0 ]; then
     echo 'sharing wlan0 to eth0'
     sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE 
     sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT 
     sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
   elif [ $# -eq 1 ]; then
     echo "sharing $1 to eth0"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i eth0 -o $1 -j ACCEPT
   elif [ $# -eq 2 ]; then
     echo "sharing $1 to $2"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o $2 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i $2 -o $1 -j ACCEPT
   fi
}

function set_bash {
   PROMPT_COMMAND='history -a'
   history -a

   # sorting in old style
   LC_COLLATE="C"
   export LC_COLLATE
   
   ulimit -c unlimited
   export HISTTIMEFORMAT="%d/%m/%y %T "
}

set_levering_up_base
setup_levering_up
set_ros
set_bash

exec "$@"

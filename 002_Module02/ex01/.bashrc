#
# ~/.bashrc
#

# If not running interactively, don't do anything
[[ $- != *i* ]] && return

alias ls='ls --color=auto'
alias grep='grep --color=auto'
PS1='[\u@\h \W]\$ '

alias obama='~/Scripts/obama.sh'
alias biden='~/Scripts/biden.sh'

if (env | grep -Fq 'DISTROBOX'); then  
	source /opt/ros/humble/setup.bash 
	export ROS_LOCALHOST_ONLY=1
	source ~/ros2_ws/install/local_setup.bash 
fi



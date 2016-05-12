#straight down (This is the start pose)

rosservice call -- /robot1_SetCartesian 407 -35 250 0 0.7071 0.7071 0

#rotate
rosservice call -- /robot1_SetCartesian 407 -85 250 0 0.7071 -0.7071 0

#tilt
rosservice call -- /robot1_SetCartesian 407 -30 240 0.1227 -0.6963 -0.6963 -0.1227

#tilt less
rosservice call -- /robot1_SetCartesian 407 -20 240 0.0922 -0.7010 -0.7010 -0.0922
#----------------------------------------------------------------------------------#

#straight down (fing opening 68)
rosservice call -- /robot1_SetCartesian 407 -30 250 0 0.7071 0.7071 0


#new tilt (fing opening 29)
rosservice call -- /robot1_SetCartesian 407 -31 240 0.1227 -0.6963 -0.6963 -0.1227

#straight down (fing opening 68)
rosservice call -- /robot1_SetCartesian 407 -30 250 0 0.7071 0.7071 0

# move back the ext finger more showing it has wider range
rosservice call -- /robot1_SetCartesian 407 -50 250 0 0.7071 0.7071 0

#rot start
rosservice call -- /robot1_SetCartesian 407 -40 250 0 0.7071 0.7071 0

#one rot end
rosservice call -- /robot1_SetCartesian 407 -40 250 0 0.3827 0.9239 0

#other rot end
rosservice call -- /robot1_SetCartesian 407 -40 250 0 0.9239 0.3827 0

#then do wavy motion = combination of rot and trans ( may involve some tilt)

#Oct29

#Go Down
rosservice call -- /robot1_SetCartesian 398 -25 288 0 0.7071 0.7071 0

#close to the PP
rosservice call -- /robot1_SetCartesian 398 15 288 0 0.7071 0.7071 0

#Come back
rosservice call -- /robot1_SetCartesian 398 -15 288 0 0.7071 0.7071 0

#Go To joint 6 = 0
rosservice call -- /robot1_SetJoints -2.16 30.02 14.92 0 45.06 0

#Go To joint 6 = 360
rosservice call -- /robot1_SetJoints -2.16 30.02 14.92 0 45.06 360

#Go To joint 6 = 0
rosservice call -- /robot1_SetJoints -2.16 30.02 14.92 0 45.06 0

# slow speed
rosservice call -- /robot1_SetSpeed 60 25

#Go To joint 6 = 90 
rosservice call -- /robot1_SetJoints -2.16 30.02 14.92 0 45.06 90

# fast speed
rosservice call -- /robot1_SetSpeed 350 100

#Go To joint 6 = 270
rosservice call -- /robot1_SetJoints -2.16 30.02 14.92 0 45.06 270

# slow speed
rosservice call -- /robot1_SetSpeed 60 25

#Go To joint 6 = 360
rosservice call -- /robot1_SetJoints -2.16 30.02 14.92 0 45.06 360



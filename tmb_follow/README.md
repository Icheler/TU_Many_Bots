# Follower Package
This package is intended to perform the subroutine of the robots guiding the blind robot 

## To Call The Follow Services via Terminal:
Call **rosservice call /follow_rob1** for robot1 to be the guiding and **rosservice call /follow_rob2** for robot2
to activate the other robot to follow the blind robot simply use then : **rosservice call /follow_blind**


## ROSPY Service Client
Make sure to add this to the base algorithm, after the conditions for the following routine come in place.
```
rospy.wait_for_service('ENTER SERVICE NAME HERE')

# Create the connection to the service. Remember it's a Trigger service
service_XX = rospy.ServiceProxy('ENTER SERVICE NAME HERE', Trigger)

# Create an object of the type TriggerRequest.
req = TriggerRequest()

# Now send the request through the connection
result = service_XX(req)  #### YOU DO NOT NEED THE VALUE IN result BUT CALL IT EITHER WAY LIKE THAT

## TURNING OFF THE FOLLOW ROUTINE ISNT SUPPORTED ##
```



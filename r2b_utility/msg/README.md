# r2b_utility/msg

## About

Describes custom messages for intuitive control and custom robot status utility.

## About Messages

1. EeConnectionStatus
	```
	# Number of the box (1|2|3) that can be connected with the EE frame
	uint8 can_connect_to

	# Whether "connect" command is sent
	bool connection_rqst_sent
	```
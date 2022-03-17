# r2b_utility/msg

## About

Describes custom messages for intuitive control and custom robot status utility.

## About Messages

1. EeConnectionStatus
	```
	# Euclidean distance between EE frame and box 1's frame
	float distance_from_box1

	# Euclidean distance between EE frame and box 2's frame
	float distance_from_box2

	# Euclidean distance between EE frame and box 1's frame
	float distance_from_box3

	# Number of the box (1|2|3) that can be connected with the EE frame
	uint8 can_connect_to
	```
## Gripper Message
## Used by gripper class to command actuation in the gripper, which is mapped to a specific
## Output in the mixer module stage

uint64 timestamp

int8 command		# Commanded state for the gripper
int8 COMMAND_GRAB = 0
int8 COMMAND_RELEASE = 1

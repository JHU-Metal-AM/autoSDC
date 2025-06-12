Create a wrapping function that waits for the responses of a *group* of commands:
- Send all commands without waiting
- Wait until responses of each command have gotten back before releasing

Controller.queue_command(device, command, num_lines, timeout, context, print_response)


demo2_refactored:
- After running the pumps and before disengaging the head, run the send in reverse and the return in original direction for ~5 seconds to clear the head of fluid

demo3: Measure multiple spots
- after disengaging head to offset, shift y by a set amount, repeat measurement

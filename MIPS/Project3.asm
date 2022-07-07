# Tyler Rose
# Project 03: CS 286
# This MIPS program is a continuation of project 2 (image compression)


##################################
#
# $s0 = word
# $s1 = offset
# $s2 = width
# $s3 = height
# $s4 = count
# $s5 = total width
# $s6 = iso pixel
# $s7 = pixel total
# $t1 = total colors in a sequence
# $t2 = previous iso pixel for comparing
# $t3 = array index
# $t4 = first 4 bytes of the eventual stored data
# $t5 = sequence incriment count
#
##################################


.data

newLine:	.asciiz "\n"
space:		.asciiz "  " 
finish:		.asciiz "\n\nCongratulations, your compression is complete."


size:			.word 8 8
image:			.word 0xffffff00 0x00ffffff
			.word 0xffff00ff 0xff00ffff
			.word 0xff00ff00 0x00ff00ff
			.word 0x00ff0000 0x000000ff
			.word 0x00ff00ff 0xff00ff00
			.word 0xff00ff00 0x00ff00ff
			.word 0xffff00ff 0xff00ffff
			.word 0xffffff00 0x00ffffff
			
output:			.space 10000			

.text

main:		
	
		lw $s0, image($s1)					# Offsets image
		lw $s2, size						# Loads the image width into register $s2
		li $t0, 4						# Loads 4 into register $t0
		lw $s3, size($t0)					# Loads the image height into register $s3
		
		li $t1, 0						# Loads zero into $t1
		li $t2, 256						# Loads 256 into $t2
		li $t3, -4						# Assists in zeroing out $t3 (array index)
		li $t5, 0						# Zeroing out the count for array
		j pixelIsolation					# Jump to shiftAndBranch				
	
increaseSequence:
		add $t1, $t1, 1 					# Increases the number of colors in a sequence by 1
		j shiftPixel

resetSequence:
		li $t1, 1						# Sets sequence count back to 1
				
shiftPixel:
		move $t2, $s6						# Copies the integer from $s6 into $t2 for comparing the next integer
		sll $s0, $s0, 8						# Shifts to the next pixel
 		add $s4, $s4, 1						# Increases the count by 1
 		li $t0, 4						# Loads 4 into register $t0
 		bne $s4, $t0, pixelIsolation				# Branches if the count is not 4 
 		add $s1, $s1, 4						# Goes to the next word
		li $s4, 0						# Sets the pixel count back to zero
		lw $s0, image($s1)					# Offsets image
	
pixelIsolation: 												
		beq $s2, $s5, array					# Beq if the width and the total width are equal (if it has reached the end of a word)
		add $s5, $s5,1						# Increases the width count by 1
		li $t0, 0xff000000					# Anything ANDed  with ff is itself, 00 is 00
		and  $s6, $s0, $t0					# Secludes the specific pixel																																															# Secludes a pixel by itself
		srl  $s6, $s6, 24					# Shifts the 8 MSB to the right, which makes them the 8 LSBs  
		
		beq $t1, 0, increaseSequence				# Branches back to the beginning of shiftAndBranch if the color sequence count is 0 
		beq $s6, $t2, increaseSequence 				# Branches back to the beginning of shiftAndBranch if the 2 previous integers are the same

array:		
		beq $t5, 1, secondShift					# Branching if the sequence incriment count is 1
		addi $t3, $t3, 4					# Incriment the index by 4
		addi $t5, $t5, 1					# Incrimenting sequence count
		
		sll $t1, $t1, 24					# Shifts the 8 LSB back the MSB position
		sll $t2, $t2, 16					# Shifts the 8 LSB back the MSB position
		
		or $t4, $t1, $t2					# Putting the integers together
		
		srl $t1, $t1, 24 					# Adjusts the value back LSB position
		srl $t2, $t2, 16					# Adjusts the value back LSB position
		
		beq $t5, 1, branchOrJump				# Branching if the sequence incriment count is 1
		
secondShift:
		sll $t1, $t1, 8						# Shifts the 8 LSB back the MSB position
		sll $t2, $t2, 0						# Shifts the 8 LSB back the MSB position	
		or $t5, $t1, $t2					# Putting the integers together
		
store:		
		or $t1, $t4, $t5					# ORing the contents of $t4 and $t5 to fit within the width of 8
		sw $t1, output($t3)					# Stores the integer in $t1 into the array (output)
		li $t5, 0						# Zero's out the sequence incriment count
		
branchOrJump:
		beq $s2, $s5, breakOff					# Beq if the width and the total width are equal (if it has reached the end of a word)
		j resetSequence						# Jumps to resetSequence
		
breakOff:
		add $s7, $s7, 1						# Increases the pixel total by 1 
		beq $s7, $s3, end					# Branches to end if pixel total and the height are equal
		li $s5, 0						# Sets total count back to zero since a new line has been printed
		li $t1, 0						# Sets sequence count back to zero 
		j pixelIsolation					# Jumps back to shiftAndBranch
				
end:
		li $v0, 4
 		la $a0, finish						# Prints the ending message indicating the program is finished running
 		syscall

		li $v0 10
		syscall							# Terminates the program
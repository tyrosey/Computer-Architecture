# Tyler Rose
# Project 02: CS 286
# This MIPS program takes an image input and compresses it


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
#
##################################


.data

newLine:	.asciiz "\n"
at:		.asciiz "@" 						# for 00 through 19 (MSB 000)
pound:		.asciiz "#" 						# for 20 through 39 (MSB 001)
asterisk:	.asciiz "*" 						# for 40 through 59 (MSB 010)
plus:		.asciiz "+" 						# for 60 through 79 (MSB 011)
equal:		.asciiz "=" 						# for 80 through 99 (MSB 100)
underscore:	.asciiz "_" 						# for a0 through bf (MSB 101)
dash:		.asciiz "-" 						# for c0 through df (MSB 110)
period:		.asciiz "." 						# for e0 through ff (MSB 111)

finish:		.asciiz "\n\nThe image is now finished.\nSelf-destructing in 5...4...3...2...1..."

size:		.word 8 8
image:		.word 0xffffff00 0x00ffffff
		.word 0xffff00ff 0xff00ffff
		.word 0xff00ff00 0x00ff00ff
		.word 0x00ff0000 0x000000ff
		.word 0x00ff00ff 0xff00ff00
		.word 0xff00ff00 0x00ff00ff
		.word 0xffff00ff 0xff00ffff
		.word 0xffffff00 0x00ffffff


.text
main:		
		lw $s0, image($s1)					# Offsets image
		lw $s2, size						# Loads the image width into register $s2
		li $t0, 4						# Loads 4 into register $t0
		lw $s3, size($t0)					# Loads the image height into register $s3
	
shiftAndBranch: 							
		beq $s2, $s5, breakOff					# Beq if the width and the total width are equal (if it has reached the end of a word)
		add $s5, $s5,1						# Increases the width count by 1
		li $t0, 0xff000000					# Anything ANDed  with ff is itself, 00 is 00
		and  $s6, $s0, $t0					# Secludes the specific pixel																																															
		srl  $s6, $s6, 29					# Shifts the 3 MSB to the right, which makes them the 3 LSBs  
		
		li $t0, 0						# Loads $t0 with an integer to beq with the new LSBs (repeated for the following instructions)
		beq $s6, $t0, print0					# Branches to the appropriate printing label according to the new LSBs (repeated for the following instructions)
		
		li $t0, 1
		beq $s6, $t0, print1
		
		li $t0, 2
		beq $s6, $t0, print2
		
		li $t0, 3
		beq $s6, $t0, print3
		
		li $t0, 4
		beq $s6, $t0, print4
		
		li $t0, 5
		beq $s6, $t0, print5
		
		li $t0, 6
		beq $s6, $t0, print6
		
		li $t0, 7
		beq $s6, $t0, print7
		
breakOff:
		add $s7, $s7, 1						# Increases the pixel total by 1 
		beq $s7, $s3, end					# Branches to end if pixel total and the height are equal
		li $v0, 4						# Loads op code 4 (Print string) into $v0
		la $a0, newLine						# Loads newLine address
		syscall							# Prints a new line
		li $s5, 0						# Sets total count back to zero since a new line has been printed
		j shiftAndBranch					# Jumps back to shiftAndBranch
		
print0:									# Prints the @ symbol
 		li $v0, 4						 
 		la $a0, at
 		syscall							
 		j endPrint
print1:									# Prints the # symbol
 		li $v0, 4
 		la $a0, pound
 		syscall							
 		j endPrint
print2:									# Prints the * symbol
 		li $v0, 4
 		la $a0, asterisk
 		syscall							
 		j endPrint
print3:									# Prints the + symbol
 		li $v0, 4	
 		la $a0, plus
 		syscall							
 		j endPrint
print4:									# Prints the = symbol
 		li $v0, 4
 		la $a0, equal
 		syscall							
 		j endPrint
print5:									# Prints the _ symbol
 		li $v0, 4
 		la $a0, underscore
 		syscall							
 		j endPrint
print6:									# Prints the - symbol
 		li $v0, 4
 		la $a0, dash
 		syscall							
 		j endPrint
print7:									# Prints the . symbol
 		li $v0, 4
 		la $a0, period
 		syscall							
 		j endPrint	
 	
endPrint:
 		sll $s0, $s0, 8						# Shifts the word back to its original place
 		add $s4, $s4, 1						# Increases the count by 1
 		li $t0, 4						# Loads 4 into register $t0
 		bne $s4, $t0, shiftAndBranch				# Branches if the count is not 4 
 		add $s1, $s1, 4						# Goes to the next word
		li $s4, 0						# Sets the pixel count back to zero
		lw $s0, image($s1)					# Offsets image
		j shiftAndBranch					# Jumps to shiftAndBranch
 	
end:
		li $v0, 4
 		la $a0, finish						# Prints the ending message indicating the program is finished running
 		syscall

		li $v0 10
		syscall							# Terminates the program
	
	
	
	
	

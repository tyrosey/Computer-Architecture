# Tyler Rose
# Project 04: CS 286
# This MIPS program takes a string as user input and determines if it is a palindrome

##################################
#
# $a0 = input
# $a1 = size of input
# $s0 = for comparing a byte
# $s1 = for comparing a byte
# $t0 = address of string/bytes of string
# $t1 = address of string/bytes of string
# $t2 = for loading/comparing bytes
#
##################################

.data

one:		.asciiz	"1"
zero:		.asciiz	"0"
prompt:		.asciiz	"Enter a string: "
memorySpace:	.space 1024 					# 1024 bytes, memory for the string

.text

main:													
	la $a0, prompt 						# Prompt user to enter a string
	li $v0, 4
	syscall
	
	la $a0, memorySpace 					# Store input
	li $a1, 1024 						# Size of string
	li $v0, 8 						# Load op code for user input
	syscall

	la $t0, memorySpace   					# Loads address of string
	la $t1, memorySpace  					# Loads address of string
  
incrementCheck:
	lb $t2, ($a0)						# Loads a byte from the address

	bgt $t2, 96, storeAndIncrement 				# If $t2 > 96 branch to storeAndIncrement
	bgt $t2, 91, increment 					# If $t2 > 91 branch to increment
	bgt $t2, 64, storeAndIncrement				# If $t2 > 64 branch to storeAndIncrement
	blt $t2, 47, testForPal					# If $t2 < 47 branch to testForPal
	
increment:
	addi $a0, $a0, 1 					# Increment the string 
	j incrementCheck					# Jump to incrementCheck and repeats process

storeAndIncrement:						# If $t2 < 58
	sb $t2, ($t0)						# Stores a byte from $t2 into memory address in $t0
	addi $a0, $a0, 1 					# Increment the string address
	addi $t0, $t0, 1					# Increment the string
	j incrementCheck					# Jump to incrementCheck

testForPal:
	sb $zero, ($a0)						# Stores zero in the character's spot
	sub $t0, $t0, 1 					# Decriment by one from the end of $t0
	j compare						# Jump to compare
		
branchTest: 
	beq $t1, $t0, palindrome				# If at middle, branch to palindrome
	sub $t0, $t0, 1						# $t0 could be 'one' off from middle (if string has odd amount of characters)(takes care of decrementing $t0)
	beq $t1, $t0, palindrome				# If at middle, branch to palindrome
	addi $t1, $t1, 1 					# Increment by one from beginning of $t1

compare:   
	lb $s0, ($t1)						# Loading byte from $t1
	lb $s1, ($t0)   					# Loading byte from $t0
	beq $s0, $s1, branchTest				# Branching if the bytes are equal
	
	la $a0, zero	 					# If bytes are not equal, then it is not a palindrome and will print a zero
	li $v0, 4
	syscall							# Print zero
	j end							# Jumps to end

palindrome: 							# Prints a 1 if the string IS a palindrome						
	li $v0, 4
	la $a0 one
	syscall
  
end: 								# Terminate program
	li $v0, 10
	syscall 


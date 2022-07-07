# Tyler Rose
# 1/29/2020
#Project 01: Hello CS 286
#This MIPS program prompts users to enter their name, then displays a welcome message using the input from the user




 .data							# submitting data now											

 prompt:	.asciiz  "Enter your name:"	# address for storing this string
 message1:	.asciiz  "Hello,  "			# address for storing this string
 message2:	.asciiz "Welcome to Hell!"		# address for storing this string
 userInput: 	.space 40				# 40 bytes, memory for the user's name

 .text
 main:
        						# Prompt the user to enter their name
        
 	li $v0, 4						# loads op code 4 into register $v0 for print string
 	la $a0, prompt						# loads prompt address into argument register 0 from memory
 	syscall						# prints the string located in $a0
       
          						# Getting the input info from user
 	       
	la $a0, userInput					# sets $a0 as the address for their name
	li $v0, 8						# loads the users input into register $v0
	la $a1, userInput						# length of space so that memory limit is not exceeded
	syscall						# prints the string located in $a0

        						# Displays "Hello, "

 	li $v0, 4						# loads op code 4 into register $v0 for print string
 	la $a0, message1					# loads message1 address into argument register 0 from memory
 	syscall						# prints the string located in $a0
        
       							# Displays the user's name
        
 	li $v0, 4						# loads op code 4 into register $v0 for print string
 	la $a0, userInput					# loads userInput address into argument register 0 from memory
 	syscall						# prints the string located in $a0
   
   							# Displays "Welcome to SIUE!"
        
 	li $v0, 4						# loads op code 4 into register $v0 for print string
 	la $a0, message2					# loads message2 address into argument register 0 from memory
 	syscall							# prints the string located in $a0
    
   							# Program is finished running
   
 	li $v0, 10						# loads op code 10 into $v0 for exiting the program
 	syscall						# ends program

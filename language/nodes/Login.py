idList = []
nameList = [] 

with open('id.txt') as f:
	for line in f: 
		row = line.split(' ')
		idList.append(row[0])
		nameList.append(row[1])
		
def activateLogin():

	msg = "\nPlease enter your ID number:\n"
	print(msg)

	count = 0
	maxCount = 3
	while count < maxCount:
		
		ans = input()
		if ans in idList:
			name = nameList[idList.index(ans)]
			msg = '\nHello ' + name		
			print(msg)
			return True
		else:
			count += 1
			if count < 3:
				msg = '\nLogin failed. Please try again:\n'
				print(msg)
	
	return False

activateLogin()


		

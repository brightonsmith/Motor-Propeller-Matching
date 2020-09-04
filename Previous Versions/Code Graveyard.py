choose_propeller =  input("If you would like to read from a propeller database, enter \'y\'. Otherwise, enter \'n\'.\n"
                          "Read from a propeller database: ").lower()
while choose_propeller ==
if choose_propeller == 'y':
    print("These are the filenames in the folder you selected: ")
    # Display all available filenames
    filename = input('Enter the full filename you would like to read from: ')
    propeller = read_propeller_data(filename)
elif choose_propeller == 'n':
    pass
else:
    choose_propeller = input("You did not enter a valid choice. Enter \'y\' or \'n\'.\n"
          "").lower()
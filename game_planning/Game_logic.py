import numpy as np
import random

turn_over = False

## Initialize client node where we would extract the game state value
# Filler values for hand dominoes and their positions
hand_dom = np.array([[1,1,4,4,1,4,1],
                     [4,4,1,1,1,4,4]])
hand_pos_cm = np.array([[850,855,851,852,853,855,850],
                        [100,150,200,250,300,350,400]])
while not turn_over: 
    # Filler values for board dominoes and their positions
    board_dom = np.array([[3,6,2,0,2,3,2],
                        [5,3,6,2,2,6,0]])
    board_pos_cm = np.array([[850,855,851,852,853,855,850],
                        [100,150,200,250,300,350,400]])

    hand_size = np.size(hand_dom,1)
    board_size = np.size(board_dom,1)

    adjacent_domino = np.zeros(2)
    played_domino = np.zeros(2)
    played_domino_position = np.zeros(2)

    i = 0
    j = 0
    match_found = False #indication that no match has been found

    while i < hand_size:
        potential_domino = hand_dom[:,i]
        top_half = potential_domino[0]
        bottom_half = potential_domino[1]
        j = 0
        while j < board_size:
            #if turn_state == 0:
                # Checks current domino in hand to every domino on the board
            if top_half == board_dom[0, j] or top_half == board_dom[1, j] or \
                    bottom_half == board_dom[0, j] or bottom_half == board_dom[1, j]:
                
                ## Input feasibility code to see if we can place domino there

                adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]]) #domino we are going to place our domino next to
                played_domino = np.array([top_half,bottom_half]) #domino that we are going to place down
                played_domino_position = hand_pos_cm[:,i]
                match_found = True #Indicates that a match has been found

                print("Board Domino is ",  adjacent_domino)
                print("Played Domino is ", played_domino)
                print("The Domino we will pick up is at", played_domino_position)

                #Include code to pick up domino from the domino position, and play it in a feasible location

                turn_over = True
                break
            else:
                j += 1
        if match_found:
            break
        i +=1
    if not match_found:
        #We need to add code relating to picking up a domino from the boneyard
        print("No match found, picking domino from the boneyard")

        #Placeholder for picking up a random domino
        new_random_dominoes = np.random.randint(1, 7, size=(2, 1))
        new_random_domino_pos = np.random.randint(150,900, size =(2,1))

        # Concatenate the random array with the existing hand_dom array
        hand_dom = np.concatenate((hand_dom, new_random_dominoes), axis=1)
        hand_pos_cm = np.concatenate((hand_pos_cm, new_random_domino_pos), axis=1)
        print(hand_dom)
else:
    print('It is now the player turn')

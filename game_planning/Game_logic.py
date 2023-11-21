import numpy as np

## Initialize client node where we would extract the game state value, specifically the hand
# Filler values for hand dominoes and their positions
hand_dom = np.array([[1,5,4,0,2,3,1],
                     [4,2,6,2,1,5,2]])
hand_pos_cm = np.array([[850,855,851,852,853,855,850],
                    [100,150,200,250,300,350,400]])

# Filler values for board dominoes and their positions
board_dom = np.array([[3,6,2,0,2,3,2],
                      [5,3,6,2,2,6,0]])
board_pos_cm = np.array([[850,855,851,852,853,855,850],
                    [100,150,200,250,300,350,400]])

hand_size = np.size(hand_dom,1)
board_size = np.size(board_dom,1)

i = 0
j = 0

while i < hand_size:
    potential_domino = hand_dom[:,i]
    top_half = potential_domino[0]
    bottom_half = potential_domino[1]
    while j < board_size:
        #if turn_state == 0:
            # Checks current domino in hand to every domino on the board
        if top_half == board_dom[0,j]:
            ## Input feasibility code to see if we can place domino there
            turn_state = 1 #Signal that turn is done
            adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]]) #domino we are going to place our domino next to
            played_domino = np.array([top_half,bottom_half]) #domino that we are going to place down
            played_domino_position = hand_pos_cm[:,i]
            j = board_size +1
        elif top_half == board_dom[1,j]:
            ## Input feasibility code to see if we can place domino there
            turn_state = 1 #Signal that turn is done
            adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]])
            played_domino = np.array([top_half,bottom_half])
            played_domino_position = hand_pos_cm[:,i]
            j = board_size +1
        elif bottom_half == board_dom[0,j]:
            ## Input feasibility code to see if we can place domino there
            turn_state = 1 #Signal that turn is done
            adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]])
            played_domino = np.array([top_half,bottom_half])
            played_domino_position = hand_pos_cm[:,i]
            j = board_size +1
        elif top_half == board_dom[1,j]:
            ## Input feasibility code to see if we can place domino there
            turn_state = 1 #Signal that turn is done
            adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]])
            played_domino = np.array([top_half,bottom_half])
            played_domino_position = hand_pos_cm[:,i]     
            j = board_size +1
        else:
            j = j+1
        #elif turn_state == 1:
            #break
    i = i + 1
print("Board Domino is ",  adjacent_domino)
print("Played Domino is ", played_domino)
print("The Domino we will pick up is at", played_domino_position)
import random
import numpy as np

def initialize_board():
    # Initialize a 2D array for the domino board
    return [[' ' for _ in range(8)] for _ in range(8)]

def print_board(board):
    # Print the current state of the domino board
    for row in board:
        print('|' + '|'.join(str(cell) if cell != ' ' else ' ' for cell in row) + '|')
        print('-' * 17)

def is_valid_move(board, domino, row, col, orientation):
    # Check if placing the domino at the specified position is a valid move
    if orientation == 'horizontal':
        if col + 1 >= len(board[0]):
            return False
        return board[row][col] == ' ' and board[row][col + 1] == ' '
    elif orientation == 'vertical':
        if row + 1 >= len(board):
            return False
        return board[row][col] == ' ' and board[row + 1][col] == ' '
    return False

def place_domino(board, domino, row, col, orientation):
    # Place the domino at the specified position on the board
    if orientation == 'horizontal':
        board[row][col] = domino[0]
        board[row][col + 1] = domino[1]
    elif orientation == 'vertical':
        board[row][col] = domino[0]
        board[row + 1][col] = domino[1]

def generate_domino():
    # Generate a random domino (e.g., [3, 5])
    return [random.randint(0, 6), random.randint(0, 6)]

def main():
    board = initialize_board()

    ## Initialize client node where we would extract the game state value, specifically the hand
    # Filler values for hand dominoes and their positions
    hand_dom = np.array([[1,5,4,0,2,3,1],
                     [4,2,6,2,1,5,2]])
    hand_pos_cm = np.array([[850,855,851,852,853,855,850],
                     [100,150,200,250,300,350,400]])
    
    # Filler values for board dominoes and their positions
    board_dom = np.array([[3,6,1,0,2,3,1],
                     [5,3,6,2,2,6,0]])
    board_pos_cm = np.array([[850,855,851,852,853,855,850],
                     [100,150,200,250,300,350,400]])
    
    hand_size = np.size(hand_dom,1)
    board_size = np.size(board_dom,1)

    turn_state = 0

    adjacent_domino = np.zeros(2)
    played_domino = np.zeros(2)
    
    for i in range(hand_size):
        potential_domino = hand_dom[:,i]
        top_half = potential_domino[0]
        bottom_half = potential_domino[1]
        for j in range(board_size):
            if turn_state == 0:
                print(top_half)
                # Checks current domino in hand to every domino on the board
                if top_half == board_dom[0,j]:
                    ## Input feasibility code to see if we can place domino there
                    turn_state = 1 #Signal that turn is done
                    adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]])
                    played_domino = np.array([top_half,bottom_half])
                    print("Board Domino is ",  adjacent_domino)
                    print("Played Domino is ", played_domino)
                elif top_half == board_dom[1,j]:
                    ## Input feasibility code to see if we can place domino there
                    turn_state = 1 #Signal that turn is done
                    adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]])
                    played_domino = np.array([top_half,bottom_half])
                    print("Board Domino is ",  adjacent_domino)
                    print("Played Domino is ", played_domino)
                elif bottom_half == board_dom[0,j]:
                    ## Input feasibility code to see if we can place domino there
                    turn_state = 1 #Signal that turn is done
                    adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]])
                    played_domino = np.array([top_half,bottom_half])
                    print("Board Domino is ",  adjacent_domino)
                    print("Played Domino is ", played_domino)
                elif top_half == board_dom[1,j]:
                    ## Input feasibility code to see if we can place domino there
                    turn_state = 1 #Signal that turn is done
                    adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]])
                    played_domino = np.array([top_half,bottom_half])
                    print("Board Domino is ",  adjacent_domino)
                    print("Played Domino is ", played_domino)
            #elif turn_state == 1:
                #break
        break

'''
    while True:
        print_board(board)

        # Player's turn
        player_domino = generate_domino()
        print(f"\nYour domino: {player_domino}")
        row = int(input("Enter the row to place the domino: "))
        col = int(input("Enter the column to place the domino: "))
        orientation = input("Enter orientation (horizontal/vertical): ").lower()

        if is_valid_move(board, player_domino, row, col, orientation):
            place_domino(board, player_domino, row, col, orientation)
        else:
            print("Invalid move! Try again.")
            continue

        # Computer's turn
        computer_domino = generate_domino()
        print(f"\nComputer's domino: {computer_domino}")
        computer_row = random.randint(0, 7)
        computer_col = random.randint(0, 7)
        computer_orientation = random.choice(['horizontal', 'vertical'])

        if is_valid_move(board, computer_domino, computer_row, computer_col, computer_orientation):
            place_domino(board, computer_domino, computer_row, computer_col, computer_orientation)
        else:
            print("Computer made an invalid move!")
'''
if __name__ == "__main__":
    main()
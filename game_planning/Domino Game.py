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

if __name__ == "__main__":
    main()
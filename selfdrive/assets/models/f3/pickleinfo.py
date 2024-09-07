import pickle
import sys

def print_pickle_data(filename):
    try:
        with open(filename, 'rb') as file:
            data = pickle.load(file)
            print(data)
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <filename>")
    else:
        print_pickle_data(sys.argv[1])

with open('h.c') as infile, open ('Impulse.c', 'w') as outfile:
    outfile.write(', '.join(infile.read().split('\n')) + '\n')
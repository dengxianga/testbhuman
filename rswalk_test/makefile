CC=g++
CFLAGS=-I.

TARGETS=all clean
.PHONY: $(TARGETS)

RSWALK_LIB = -L/home/nao/rswalk2014 -static -lrswalk2014

all: test_unsw

%.o: %.c 
	$(CC) -c -o $@ $< $(CFLAGS) 

test_unsw: test_unsw.o 
	g++ $(RSWALK_LIB) $< -o $@

clean:
	rm -f *.o test_unsw